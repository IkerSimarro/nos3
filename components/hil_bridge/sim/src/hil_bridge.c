/*
** NOS3 HIL Bridge
**
** Stands in for the cFS flight software container (hostname nos-fsw) so that a physical MCU,
** connected over a serial port, can act as the flight computer:
**
**   MCU <--hil_link frames--> hil_bridge --NOS Engine--> usart_N / i2c_N / spi_N / can_N --> device sims
**                                        --NOS Engine--> time bus (simulation time -> TIME frames)
**                                        --UDP-------->  trq-sim:14242 (magnetorquers)
**                                        --UDP-------->  COSMOS umbilical (TC in :9010, TM out :9011)
**                                        --UDP-------->  ground link emulator (RF in :9020, RF out :9021)
**
** Interfaces are specified in the HITL FlatSat ICD (IF-01, IF-02, section 8).
**
** Bus handling mirrors fsw/apps/hwlib/sim/src (lib*.c, nos_link.c): buses are opened lazily on first
** use with the same bus names, node name and master address, so the device sims need no changes.
**
** Threads: the main thread owns the serial link, UDP, UART polling and TIME frames. Each I2C, SPI and CAN
** bus has a worker thread that runs its transactions, because NOS Engine calls block until the simulator
** answers (its timeouts default to infinite). A stalled simulator therefore holds only its own bus; a new
** request for that bus is answered BUS_ERROR at once and every other bus carries on (FlatSat NCR-006).
*/
#define _DEFAULT_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <netdb.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <Client/CInterface.h>
#include <Uart/Client/CInterface.h>
#include <I2C/Client/CInterface.h>
#include <Spi/Client/CInterface.h>
#include <Can/Client/CInterface.h>

#include "hil_link.h"

/* Same limits and constants as fsw/apps/hwlib/sim */
#define HIL_NUM_BUSES        30
#define HIL_NOS_MASTER_ADDR  10
#define HIL_USART_QUEUE_SIZE 4096
#define HIL_NOS_NODE_NAME    "fsw"

#define HIL_UDP_MAX          65536
#define HIL_POLL_MS          2
#define HIL_LINK_TIMEOUT_S   3

#define HIL_TIME_PERIOD_S    1
#define HIL_TRQ_MAX_DUTY     10000 /* 100.00 % */

typedef struct
{
    const char *serial_dev;
    int         baud;
    const char *nos_uri;
    /* Umbilical (ICD IF-06): space packets to/from COSMOS FLATSAT_UMB */
    const char *umb_host;
    int         umb_tc_port; /* listen */
    int         umb_tm_port; /* send */
    /* Simulated RF link (ICD 7.5): RF frames to/from the ground station link emulator */
    const char *rf_host;
    int         rf_rx_port;  /* listen */
    int         rf_tx_port;  /* send */
    /* Magnetorquer sim, same UDP text protocol as hwlib libtrq */
    const char *trq_host;
    int         trq_port;
    /* NOS3 simulation time (ICD 8.2): tick count on the time bus, converted as the NOS3 sims do */
    const char *time_uri;
    const char *time_bus;
    double      start_time;  /* common.absolute-start-time, J2000 seconds */
    long        us_per_tick; /* common.sim-microseconds-per-tick */
    int         verbose;
} hil_config_t;

typedef struct
{
    const char *name;
    int         port;
    int         resolved;
    struct sockaddr_in addr;
} hil_udp_dest_t;

static volatile sig_atomic_t keep_running = 1;

/* Defaults match cfg/sims/sc-1-nos3-simulator.xml and the ICD port allocation (ICD 8.3) */
static hil_config_t cfg = {
    .serial_dev  = "/dev/ttyHIL0",
    .baud        = 921600,
    .nos_uri     = "tcp://nos-engine-server:12000",
    .umb_host    = "cosmos",
    .umb_tc_port = 9010,
    .umb_tm_port = 9011,
    .rf_host     = "flatsat-gs",
    .rf_rx_port  = 9020,
    .rf_tx_port  = 9021,
    .trq_host    = "trq-sim",
    .trq_port    = 14242,
    .time_uri    = "tcp://nos-engine-server:12001",
    .time_bus    = "command",
    .start_time  = 814254200.0,
    .us_per_tick = 10000,
    .verbose     = 0,
};

static NE_TransportHub *hub = NULL;
static NE_Bus          *time_bus = NULL;
static NE_Uart         *uart_dev[HIL_NUM_BUSES];
static NE_I2CHandle    *i2c_dev[HIL_NUM_BUSES];
static NE_SpiHandle    *spi_dev[HIL_NUM_BUSES];
static NE_CanHandle    *can_dev[HIL_NUM_BUSES];

static int serial_fd = -1;
static int umb_sock  = -1;
static int rf_sock   = -1;
static int tx_sock   = -1;
static uint8_t bridge_seq = 0;

static hil_udp_dest_t umb_dest;
static hil_udp_dest_t rf_dest;
static hil_udp_dest_t trq_dest;

/* Frame buffers are large; keep them off the stack */
static hil_frame_t rx_frame;
static hil_frame_t tx_frame;
static uint8_t     tx_wire[HIL_ENCODED_MAX];

/* serial_mu: tx_frame/tx_wire and the serial line, shared by the main thread and the bus workers.
** open_mu: creation of NOS Engine bus handles, which the main thread (OPEN frames) and workers both do. */
static pthread_mutex_t serial_mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t open_mu   = PTHREAD_MUTEX_INITIALIZER;

static time_t last_mcu_frame = 0;
static int    mcu_link_up    = 0;

static void log_msg(const char *fmt, ...)
{
    va_list ap;
    printf("hil_bridge: ");
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
}

/* Monotonic milliseconds, for timing how long NOS Engine takes to open a bus */
static double now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

/*
** NOS Engine transactions block with no timeout (e.g. if the server dies), so the main loop may never
** see keep_running go false; a second signal exits immediately.
*/
static void handle_signal(int sig)
{
    (void)sig;
    if (!keep_running)
    {
        _exit(1);
    }
    keep_running = 0;
}

/*
** Serial port
*/
static speed_t baud_to_speed(int baud)
{
    switch (baud)
    {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
        case 230400:  return B230400;
        case 460800:  return B460800;
        case 921600:  return B921600;
        case 1000000: return B1000000;
        case 2000000: return B2000000;
        default:      return 0;
    }
}

static int serial_open(const char *dev, int baud)
{
    struct termios tio;
    speed_t        speed = baud_to_speed(baud);
    int            fd;

    if (speed == 0)
    {
        log_msg("unsupported baud rate %d", baud);
        return -1;
    }

    fd = open(dev, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        log_msg("cannot open %s: %s", dev, strerror(errno));
        return -1;
    }

    if (tcgetattr(fd, &tio) == 0)
    {
        cfmakeraw(&tio);
        cfsetispeed(&tio, speed);
        cfsetospeed(&tio, speed);
        tio.c_cflag |= (CLOCAL | CREAD);
        tio.c_cc[VMIN]  = 0;
        tio.c_cc[VTIME] = 0;
        if (tcsetattr(fd, TCSANOW, &tio) != 0)
        {
            log_msg("warning: tcsetattr on %s failed: %s", dev, strerror(errno));
        }
        tcflush(fd, TCIOFLUSH);
    }
    else
    {
        /* Not a tty (e.g. a FIFO in a test); carry on with raw reads/writes */
        log_msg("warning: %s is not a tty, skipping line settings", dev);
    }
    return fd;
}

static void serial_send(uint8_t type, uint8_t bus, uint8_t seq, uint8_t status, uint32_t addr,
                        const uint8_t *payload, size_t len)
{
    size_t  n;
    size_t  off = 0;
    ssize_t w;

    if (len > HIL_MAX_PAYLOAD)
    {
        log_msg("dropping frame type 0x%02x: payload %zu > %d", type, len, HIL_MAX_PAYLOAD);
        return;
    }

    pthread_mutex_lock(&serial_mu);
    tx_frame.type   = type;
    tx_frame.bus    = bus;
    tx_frame.seq    = seq;
    tx_frame.status = status;
    tx_frame.addr   = addr;
    tx_frame.len    = (uint16_t)len;
    if (len > 0)
    {
        memcpy(tx_frame.payload, payload, len);
    }

    n = hil_encode(&tx_frame, tx_wire, sizeof(tx_wire));
    if (n == 0)
    {
        pthread_mutex_unlock(&serial_mu);
        log_msg("failed to encode frame type 0x%02x", type);
        return;
    }

    while (off < n)
    {
        w = write(serial_fd, tx_wire + off, n - off);
        if (w < 0)
        {
            if (errno == EINTR || errno == EAGAIN)
            {
                continue;
            }
            pthread_mutex_unlock(&serial_mu);
            log_msg("serial write failed: %s", strerror(errno));
            return;
        }
        off += (size_t)w;
    }
    pthread_mutex_unlock(&serial_mu);

    if (cfg.verbose)
    {
        log_msg("-> MCU type=0x%02x bus=%u seq=%u status=%u addr=0x%x len=%zu", type, bus, seq, status, addr, len);
    }
}

/*
** UDP
*/
static int udp_bind(int port)
{
    struct sockaddr_in addr;
    int                fd = socket(AF_INET, SOCK_DGRAM, 0);
    int                one = 1;

    if (fd < 0)
    {
        return -1;
    }
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons((uint16_t)port);
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0)
    {
        log_msg("cannot bind UDP port %d: %s", port, strerror(errno));
        close(fd);
        return -1;
    }
    return fd;
}

/* Resolve lazily: radio-sim may start after the bridge */
static int udp_resolve(hil_udp_dest_t *dest)
{
    struct addrinfo  hints;
    struct addrinfo *res = NULL;

    if (dest->resolved)
    {
        return 0;
    }

    memset(&hints, 0, sizeof(hints));
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    if (getaddrinfo(dest->name, NULL, &hints, &res) != 0 || res == NULL)
    {
        return -1;
    }

    memcpy(&dest->addr, res->ai_addr, sizeof(dest->addr));
    dest->addr.sin_port = htons((uint16_t)dest->port);
    dest->resolved      = 1;
    freeaddrinfo(res);
    log_msg("resolved %s:%d -> %s", dest->name, dest->port, inet_ntoa(dest->addr.sin_addr));
    return 0;
}

static void udp_send(hil_udp_dest_t *dest, const uint8_t *data, size_t len)
{
    if (udp_resolve(dest) != 0)
    {
        log_msg("cannot resolve %s, dropping %zu bytes", dest->name, len);
        return;
    }
    if (sendto(tx_sock, data, len, 0, (struct sockaddr *)&dest->addr, sizeof(dest->addr)) < 0)
    {
        log_msg("sendto %s:%d failed: %s", dest->name, dest->port, strerror(errno));
    }
}

/* Forward one datagram from a UDP socket to the MCU as the given frame type */
/* RF_RX datagrams from the link emulator start with 4 bytes of radio metadata (FlatSat ICD 7.5): RSSI i16
** dBm big-endian, SNR i8 in 0.25 dB, flags. They go into the frame's addr as the ground modem would put
** them: RSSI in bits 0-15, SNR in bits 16-23 */
#define RF_META_LEN 4

static void udp_to_mcu(int fd, uint8_t type)
{
    static uint8_t buf[HIL_UDP_MAX];
    ssize_t        n    = recv(fd, buf, sizeof(buf), 0);
    uint32_t       addr = 0;
    uint8_t       *data = buf;

    if (n <= 0)
    {
        return;
    }
    if (type == HIL_RF_RX)
    {
        if (n < RF_META_LEN)
        {
            log_msg("dropping %zd-byte RF datagram: shorter than its metadata", n);
            return;
        }
        addr = (uint32_t)(uint16_t)((buf[0] << 8) | buf[1]) | ((uint32_t)buf[2] << 16);
        data += RF_META_LEN;
        n -= RF_META_LEN;
    }
    if ((size_t)n > HIL_MAX_PAYLOAD)
    {
        log_msg("dropping %zd-byte packet for MCU (max %d); raise HIL_MAX_PAYLOAD", n, HIL_MAX_PAYLOAD);
        return;
    }
    serial_send(type, 0, bridge_seq++, HIL_STATUS_OK, addr, data, (size_t)n);
}

/*
** NOS Engine buses (lazy open, same as hwlib sim)
*/
enum
{
    WORKER_I2C,
    WORKER_SPI,
    WORKER_CAN,
    WORKER_KINDS
};

static NE_Uart *get_uart(uint8_t bus)
{
    char   name[16];
    double t0;

    if (bus >= HIL_NUM_BUSES)
    {
        return NULL;
    }
    pthread_mutex_lock(&open_mu);
    if (uart_dev[bus] == NULL)
    {
        snprintf(name, sizeof(name), "usart_%u", bus);
        t0 = now_ms();
        uart_dev[bus] = NE_uart_open3(hub, HIL_NOS_NODE_NAME, cfg.nos_uri, name, bus);
        if (uart_dev[bus] == NULL)
        {
            pthread_mutex_unlock(&open_mu);
            pthread_mutex_unlock(&open_mu);
            pthread_mutex_unlock(&open_mu);
            pthread_mutex_unlock(&open_mu);
            log_msg("failed to open %s on %s", name, cfg.nos_uri);
            return NULL;
        }
        NE_uart_set_queue_size(uart_dev[bus], HIL_USART_QUEUE_SIZE);
        log_msg("opened %s in %.0f ms", name, now_ms() - t0);
    }
    pthread_mutex_unlock(&open_mu);
    return uart_dev[bus];
}

static NE_I2CHandle *get_i2c(uint8_t bus)
{
    char   name[16];
    double t0;

    if (bus >= HIL_NUM_BUSES)
    {
        return NULL;
    }
    pthread_mutex_lock(&open_mu);
    if (i2c_dev[bus] == NULL)
    {
        snprintf(name, sizeof(name), "i2c_%u", bus);
        t0 = now_ms();
        i2c_dev[bus] = NE_i2c_init_master3(hub, HIL_NOS_MASTER_ADDR, cfg.nos_uri, name);
        if (i2c_dev[bus] == NULL)
        {
            log_msg("failed to open %s on %s", name, cfg.nos_uri);
            return NULL;
        }
        log_msg("opened %s in %.0f ms", name, now_ms() - t0);
    }
    pthread_mutex_unlock(&open_mu);
    return i2c_dev[bus];
}

/* hwlib maps SPI (bus, cs) to NOS bus spi_<bus*10+cs> */
static NE_SpiHandle *get_spi(uint8_t bus, uint8_t cs)
{
    double   t0;
    char     name[16];
    unsigned idx = (unsigned)bus * 10u + cs;

    if (cs >= 10 || idx >= HIL_NUM_BUSES)
    {
        return NULL;
    }
    pthread_mutex_lock(&open_mu);
    if (spi_dev[idx] == NULL)
    {
        snprintf(name, sizeof(name), "spi_%u", idx);
        t0 = now_ms();
        spi_dev[idx] = NE_spi_init_master3(hub, cfg.nos_uri, name);
        if (spi_dev[idx] == NULL)
        {
            log_msg("failed to open %s on %s", name, cfg.nos_uri);
            return NULL;
        }
        log_msg("opened %s in %.0f ms", name, now_ms() - t0);
    }
    pthread_mutex_unlock(&open_mu);
    return spi_dev[idx];
}

static NE_CanHandle *get_can(uint8_t bus)
{
    char   name[16];
    double t0;

    if (bus >= HIL_NUM_BUSES)
    {
        return NULL;
    }
    pthread_mutex_lock(&open_mu);
    if (can_dev[bus] == NULL)
    {
        snprintf(name, sizeof(name), "can_%u", bus);
        t0 = now_ms();
        can_dev[bus] = NE_can_init_master3(hub, HIL_NOS_MASTER_ADDR, cfg.nos_uri, name);
        if (can_dev[bus] == NULL)
        {
            log_msg("failed to open %s on %s", name, cfg.nos_uri);
            return NULL;
        }
        log_msg("opened %s in %.0f ms", name, now_ms() - t0);
    }
    pthread_mutex_unlock(&open_mu);
    return can_dev[bus];
}

static int worker_busy(int kind, int idx);

static void close_buses(void)
{
    int i;

    /* A bus whose worker is stuck in a transaction is left open: closing it could block */
    for (i = 0; i < HIL_NUM_BUSES; i++)
    {
        if (uart_dev[i]) NE_uart_close(&uart_dev[i]);
        if (i2c_dev[i] && !worker_busy(WORKER_I2C, i)) NE_i2c_close(&i2c_dev[i]);
        if (spi_dev[i] && !worker_busy(WORKER_SPI, i)) NE_spi_close(&spi_dev[i]);
        if (can_dev[i] && !worker_busy(WORKER_CAN, i)) NE_can_close(&can_dev[i]);
    }
}

/* Push any bytes the device sims wrote to open USARTs up to the MCU */
static void poll_uarts(void)
{
    static uint8_t buf[HIL_MAX_PAYLOAD];
    size_t         avail;
    size_t         n;
    int            i;

    for (i = 0; i < HIL_NUM_BUSES; i++)
    {
        if (uart_dev[i] == NULL)
        {
            continue;
        }
        while ((avail = NE_uart_available(uart_dev[i])) > 0)
        {
            n = NE_uart_read(uart_dev[i], buf, avail < sizeof(buf) ? avail : sizeof(buf));
            if (n == 0)
            {
                break;
            }
            serial_send(HIL_UART_RX, (uint8_t)i, bridge_seq++, HIL_STATUS_OK, 0, buf, n);
        }
    }
}

/*
** Magnetorquers: the torquer sim isn't on a NOS Engine bus; hwlib's libtrq sends it UDP text
** "<index> <duty %>\n" with a signed duty, so the bridge sends exactly that
*/
#define HIL_NUM_TORQUERS 3

static void handle_trq(const hil_frame_t *f)
{
    char    msg[64];
    int16_t duty;
    int     n;

    if (f->len != 2 || f->bus >= HIL_NUM_TORQUERS)
    {
        log_msg("TRQ_CMD: bad request (torquer %u, %u payload bytes)", f->bus, f->len);
        return;
    }
    duty = (int16_t)hil_get_u16(f->payload);
    if (duty < -HIL_TRQ_MAX_DUTY || duty > HIL_TRQ_MAX_DUTY)
    {
        log_msg("TRQ_CMD: duty %d out of range", duty);
        return;
    }
    n = snprintf(msg, sizeof(msg), "%u %f\n", f->bus, duty / 100.0);
    udp_send(&trq_dest, (const uint8_t *)msg, (size_t)n);
}

/*
** Simulation time: tick count from the NOS3 time bus, converted the way the NOS3 sims do
** (absolute-start-time + ticks * sim-microseconds-per-tick), sent as CUC seconds + 2^-16 subseconds
*/
static void send_time(void)
{
    NE_SimTime ticks;
    double     t;
    uint32_t   sec;
    uint16_t   sub;
    uint8_t    p[6];

    if (time_bus == NULL)
    {
        return;
    }
    ticks = NE_bus_get_time(time_bus);
    t     = cfg.start_time + (double)ticks * (double)cfg.us_per_tick / 1e6;
    sec   = (uint32_t)t;
    sub   = (uint16_t)((t - (double)sec) * 65536.0);

    p[0] = (uint8_t)(sec & 0xFF);
    p[1] = (uint8_t)((sec >> 8) & 0xFF);
    p[2] = (uint8_t)((sec >> 16) & 0xFF);
    p[3] = (uint8_t)((sec >> 24) & 0xFF);
    hil_put_u16(&p[4], sub);
    serial_send(HIL_TIME, 0, bridge_seq++, HIL_STATUS_OK, 0, p, sizeof(p));
}

/*
** Request handling
*/

/* I2C/SPI/CAN requests share a payload layout: rxlen u16 LE, then the bytes to write */
static int parse_txn(const hil_frame_t *f, const uint8_t **tx, size_t *txlen, size_t *rxlen)
{
    if (f->len < 2)
    {
        return -1;
    }
    *rxlen = hil_get_u16(f->payload);
    *tx    = f->payload + 2;
    *txlen = (size_t)f->len - 2;
    return (*rxlen <= HIL_MAX_PAYLOAD) ? 0 : -1;
}

/* A transaction's reply, built on the worker thread and sent once the worker is idle again */
typedef struct
{
    uint8_t  type;
    uint8_t  bus;
    uint8_t  seq;
    uint8_t  status;
    uint32_t addr;
    size_t   len;
    uint8_t  data[HIL_MAX_PAYLOAD];
} txn_reply_t;

/* Runs one transaction and builds its reply; called on the bus's worker thread */
static void execute_txn(const hil_frame_t *f, txn_reply_t *r)
{
    uint8_t       *rx = r->data;
    const uint8_t *tx;
    size_t         txlen;
    size_t         rxlen;
    uint8_t        status = HIL_STATUS_BUS_ERROR;

    r->type = (uint8_t)(f->type + 1);
    r->bus  = f->bus;
    r->seq  = f->seq;
    r->addr = f->addr;
    r->len  = 0;
    if (parse_txn(f, &tx, &txlen, &rxlen) != 0)
    {
        r->status = HIL_STATUS_BAD_REQ;
        return;
    }
    memset(rx, 0, rxlen);

    switch (f->type)
    {
        case HIL_I2C_TXN:
        {
            NE_I2CHandle *dev = get_i2c(f->bus);
            if (dev == NULL)
            {
                status = (f->bus >= HIL_NUM_BUSES) ? HIL_STATUS_BAD_REQ : HIL_STATUS_BUS_ERROR;
            }
            else if (txlen == 0 && rxlen == 0)
            {
                status = HIL_STATUS_OK; /* same shortcut as hwlib i2c_master_transaction */
            }
            else if (NE_i2c_transaction(dev, (uint16_t)f->addr, tx, txlen, rx, rxlen) == NE_I2C_SUCCESS)
            {
                status = HIL_STATUS_OK;
            }
            break;
        }
        case HIL_SPI_TXN:
        {
            NE_SpiHandle *dev = get_spi(f->bus, (uint8_t)f->addr);
            if (dev == NULL)
            {
                status = (f->addr >= 10) ? HIL_STATUS_BAD_REQ : HIL_STATUS_BUS_ERROR;
            }
            else
            {
                NE_spi_select_chip(dev, (uint8_t)f->addr);
                if (NE_spi_transaction(dev, tx, txlen, rx, rxlen) == NE_SPI_SUCCESS)
                {
                    status = HIL_STATUS_OK;
                }
                NE_spi_unselect_chip(dev);
            }
            break;
        }
        case HIL_CAN_TXN:
        {
            NE_CanHandle *dev = get_can(f->bus);
            if (dev == NULL)
            {
                status = (f->bus >= HIL_NUM_BUSES) ? HIL_STATUS_BAD_REQ : HIL_STATUS_BUS_ERROR;
            }
            else if (NE_can_transaction(dev, f->addr, tx, txlen, rx, rxlen) == NE_CAN_SUCCESS)
            {
                status = HIL_STATUS_OK;
            }
            break;
        }
        default:
            status = HIL_STATUS_UNKNOWN;
            break;
    }

    r->status = status;
    r->len    = (status == HIL_STATUS_OK) ? rxlen : 0;
}

/*
** Bus workers (NCR-006)
*/
typedef struct
{
    pthread_mutex_t mu;
    pthread_cond_t  cv;
    int             started;
    int             busy; /* a transaction is queued or running */
    hil_frame_t     job;
} bus_worker_t;

static bus_worker_t workers[WORKER_KINDS][HIL_NUM_BUSES];

static void *worker_main(void *arg)
{
    bus_worker_t *w = arg;
    txn_reply_t   reply;

    for (;;)
    {
        pthread_mutex_lock(&w->mu);
        while (!w->busy)
        {
            pthread_cond_wait(&w->cv, &w->mu);
        }
        pthread_mutex_unlock(&w->mu);

        /* The job isn't touched by the main thread while busy is set */
        execute_txn(&w->job, &reply);

        /* Idle before replying (NCR-011): the MCU sends its next request on this bus as soon as it has the reply,
           and on a loaded machine this thread can be descheduled between the two */
        pthread_mutex_lock(&w->mu);
        w->busy = 0;
        pthread_mutex_unlock(&w->mu);
        serial_send(reply.type, reply.bus, reply.seq, reply.status, reply.addr, reply.data, reply.len);
    }
    return NULL;
}

/* Worker for a transaction frame, or NULL if the bus number is out of range */
static bus_worker_t *worker_for(const hil_frame_t *f)
{
    unsigned idx;

    switch (f->type)
    {
        case HIL_I2C_TXN:
            return f->bus < HIL_NUM_BUSES ? &workers[WORKER_I2C][f->bus] : NULL;
        case HIL_SPI_TXN:
            idx = (unsigned)f->bus * 10u + f->addr; /* hwlib maps (bus, cs) to spi_<bus*10+cs> */
            return (f->addr < 10 && idx < HIL_NUM_BUSES) ? &workers[WORKER_SPI][idx] : NULL;
        case HIL_CAN_TXN:
            return f->bus < HIL_NUM_BUSES ? &workers[WORKER_CAN][f->bus] : NULL;
        default:
            return NULL;
    }
}

/* Main thread: hand a transaction to its bus's worker without waiting for it */
static void dispatch_txn(const hil_frame_t *f)
{
    uint8_t       rsp_type = (uint8_t)(f->type + 1);
    bus_worker_t *w        = worker_for(f);
    pthread_t     thread;
    int           busy;

    if (w == NULL)
    {
        serial_send(rsp_type, f->bus, f->seq, HIL_STATUS_BAD_REQ, f->addr, NULL, 0);
        return;
    }

    pthread_mutex_lock(&w->mu);
    if (!w->started)
    {
        pthread_cond_init(&w->cv, NULL);
        if (pthread_create(&thread, NULL, worker_main, w) != 0)
        {
            pthread_mutex_unlock(&w->mu);
            log_msg("cannot start a worker thread for frame type 0x%02x bus %u", f->type, f->bus);
            serial_send(rsp_type, f->bus, f->seq, HIL_STATUS_BUS_ERROR, f->addr, NULL, 0);
            return;
        }
        pthread_detach(thread);
        w->started = 1;
    }
    busy = w->busy;
    if (!busy)
    {
        w->job  = *f;
        w->busy = 1;
        pthread_cond_signal(&w->cv);
    }
    pthread_mutex_unlock(&w->mu);

    if (busy)
    {
        /* The previous transaction on this bus is still waiting for its simulator */
        if (cfg.verbose)
        {
            log_msg("bus busy: frame type 0x%02x bus %u answered BUS_ERROR", f->type, f->bus);
        }
        serial_send(rsp_type, f->bus, f->seq, HIL_STATUS_BUS_ERROR, f->addr, NULL, 0);
    }
}

static int worker_busy(int kind, int idx)
{
    bus_worker_t *w = &workers[kind][idx];
    int           busy;

    pthread_mutex_lock(&w->mu);
    busy = w->busy;
    pthread_mutex_unlock(&w->mu);
    return busy;
}

static void init_workers(void)
{
    int k;
    int i;

    for (k = 0; k < WORKER_KINDS; k++)
    {
        for (i = 0; i < HIL_NUM_BUSES; i++)
        {
            pthread_mutex_init(&workers[k][i].mu, NULL);
            workers[k][i].started = 0;
            workers[k][i].busy    = 0;
        }
    }
}

static void handle_frame(const hil_frame_t *f)
{
    last_mcu_frame = time(NULL);
    if (!mcu_link_up)
    {
        mcu_link_up = 1;
        log_msg("MCU link up");
    }

    if (cfg.verbose)
    {
        log_msg("<- MCU type=0x%02x bus=%u seq=%u addr=0x%x len=%u", f->type, f->bus, f->seq, f->addr, f->len);
    }

    switch (f->type)
    {
        case HIL_HEARTBEAT:
            serial_send(HIL_HEARTBEAT, 0, f->seq, HIL_STATUS_OK, f->addr, f->payload, f->len);
            break;

        case HIL_LOG:
            log_msg("MCU: %.*s", (int)f->len, (const char *)f->payload);
            break;

        /* Opening a NOS Engine bus takes about 50 ms; the MCU opens its buses up front so the
         * first transaction on each doesn't pay that against its timeout */
        case HIL_UART_OPEN:
            if (get_uart(f->bus) == NULL)
            {
                log_msg("UART_OPEN: cannot open usart_%u", f->bus);
            }
            break;

        case HIL_I2C_OPEN:
            if (get_i2c(f->bus) == NULL)
            {
                log_msg("I2C_OPEN: cannot open i2c_%u", f->bus);
            }
            break;

        case HIL_SPI_OPEN:
            if (get_spi(f->bus, (uint8_t)f->addr) == NULL)
            {
                log_msg("SPI_OPEN: cannot open SPI bus %u chip select %u", f->bus, f->addr);
            }
            break;

        case HIL_CAN_OPEN:
            if (get_can(f->bus) == NULL)
            {
                log_msg("CAN_OPEN: cannot open can_%u", f->bus);
            }
            break;

        case HIL_UART_TX:
        {
            NE_Uart *dev = get_uart(f->bus);
            if (dev != NULL && f->len > 0)
            {
                NE_uart_write(dev, f->payload, f->len);
            }
            break;
        }

        case HIL_I2C_TXN:
        case HIL_SPI_TXN:
        case HIL_CAN_TXN:
            dispatch_txn(f);
            break;

        case HIL_TO_PKT:
            udp_send(&umb_dest, f->payload, f->len);
            break;

        case HIL_RF_TX:
            udp_send(&rf_dest, f->payload, f->len);
            break;

        case HIL_TRQ_CMD:
            handle_trq(f);
            break;

        default:
            log_msg("unknown frame type 0x%02x from MCU", f->type);
            serial_send(f->type, f->bus, f->seq, HIL_STATUS_UNKNOWN, f->addr, NULL, 0);
            break;
    }
}

static void read_serial(hil_decoder_t *dec)
{
    static uint8_t buf[4096];
    ssize_t        n = read(serial_fd, buf, sizeof(buf));
    ssize_t        i;

    if (n < 0 && errno != EINTR && errno != EAGAIN)
    {
        log_msg("serial read failed: %s", strerror(errno));
        keep_running = 0;
        return;
    }

    for (i = 0; i < n; i++)
    {
        hil_decode_result_t r = hil_decoder_feed(dec, buf[i], &rx_frame);
        if (r == HIL_DECODE_FRAME)
        {
            handle_frame(&rx_frame);
        }
        else if (r == HIL_DECODE_ERROR)
        {
            log_msg("discarded corrupt frame from MCU");
        }
    }
}

/*
** Main
*/
static void usage(const char *prog)
{
    printf("Usage: %s [options]\n"
           "  -d, --device PATH        serial device (default %s)\n"
           "  -b, --baud RATE          baud rate (default %d)\n"
           "  -n, --nos-uri URI        NOS Engine server for device buses (default %s)\n"
           "  -u, --uart N             open usart_N at startup (repeatable)\n"
           "      --umb-host HOST      umbilical telemetry destination, COSMOS (default %s)\n"
           "      --umb-tc-port P      UDP listen port for umbilical telecommands (default %d)\n"
           "      --umb-tm-port P      UDP port for umbilical telemetry (default %d)\n"
           "      --rf-host HOST       ground station link emulator (default %s)\n"
           "      --rf-rx-port P       UDP listen port for RF frames to the MCU (default %d)\n"
           "      --rf-tx-port P       UDP port for RF frames from the MCU (default %d)\n"
           "      --trq-host HOST      magnetorquer sim (default %s)\n"
           "      --trq-port P         magnetorquer sim UDP port (default %d)\n"
           "      --time-uri URI       NOS Engine server for the time bus, \"none\" to disable (default %s)\n"
           "      --time-bus NAME      time bus name (default %s)\n"
           "      --start-time S       absolute-start-time, J2000 seconds (default %.1f)\n"
           "      --us-per-tick N      sim-microseconds-per-tick (default %ld)\n"
           "  -v, --verbose            log every frame\n",
           prog, cfg.serial_dev, cfg.baud, cfg.nos_uri, cfg.umb_host, cfg.umb_tc_port, cfg.umb_tm_port,
           cfg.rf_host, cfg.rf_rx_port, cfg.rf_tx_port, cfg.trq_host, cfg.trq_port, cfg.time_uri, cfg.time_bus,
           cfg.start_time, cfg.us_per_tick);
}

int main(int argc, char *argv[])
{
    static const struct option long_opts[] = {
        {"device", required_argument, NULL, 'd'},
        {"baud", required_argument, NULL, 'b'},
        {"nos-uri", required_argument, NULL, 'n'},
        {"uart", required_argument, NULL, 'u'},
        {"umb-host", required_argument, NULL, 1},
        {"umb-tc-port", required_argument, NULL, 2},
        {"umb-tm-port", required_argument, NULL, 3},
        {"rf-host", required_argument, NULL, 4},
        {"rf-rx-port", required_argument, NULL, 5},
        {"rf-tx-port", required_argument, NULL, 6},
        {"trq-host", required_argument, NULL, 7},
        {"trq-port", required_argument, NULL, 8},
        {"time-uri", required_argument, NULL, 9},
        {"time-bus", required_argument, NULL, 10},
        {"start-time", required_argument, NULL, 11},
        {"us-per-tick", required_argument, NULL, 12},
        {"verbose", no_argument, NULL, 'v'},
        {"help", no_argument, NULL, 'h'},
        {NULL, 0, NULL, 0}};

    int           preopen[HIL_NUM_BUSES];
    int           num_preopen = 0;
    int           opt;
    int           i;
    time_t        last_time_sent = 0;
    hil_decoder_t dec;
    struct pollfd fds[3];
    struct sigaction sa;

    setvbuf(stdout, NULL, _IOLBF, 0);

    while ((opt = getopt_long(argc, argv, "d:b:n:u:vh", long_opts, NULL)) != -1)
    {
        switch (opt)
        {
            case 'd': cfg.serial_dev = optarg; break;
            case 'b': cfg.baud = atoi(optarg); break;
            case 'n': cfg.nos_uri = optarg; break;
            case 'u':
                if (num_preopen < HIL_NUM_BUSES)
                {
                    preopen[num_preopen++] = atoi(optarg);
                }
                break;
            case 1: cfg.umb_host = optarg; break;
            case 2: cfg.umb_tc_port = atoi(optarg); break;
            case 3: cfg.umb_tm_port = atoi(optarg); break;
            case 4: cfg.rf_host = optarg; break;
            case 5: cfg.rf_rx_port = atoi(optarg); break;
            case 6: cfg.rf_tx_port = atoi(optarg); break;
            case 7: cfg.trq_host = optarg; break;
            case 8: cfg.trq_port = atoi(optarg); break;
            case 9: cfg.time_uri = optarg; break;
            case 10: cfg.time_bus = optarg; break;
            case 11: cfg.start_time = atof(optarg); break;
            case 12: cfg.us_per_tick = atol(optarg); break;
            case 'v': cfg.verbose = 1; break;
            case 'h': usage(argv[0]); return 0;
            default: usage(argv[0]); return 1;
        }
    }

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = handle_signal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    umb_dest.name = cfg.umb_host;
    umb_dest.port = cfg.umb_tm_port;
    rf_dest.name  = cfg.rf_host;
    rf_dest.port  = cfg.rf_tx_port;
    trq_dest.name = cfg.trq_host;
    trq_dest.port = cfg.trq_port;

    log_msg("serial %s @ %d, device buses %s", cfg.serial_dev, cfg.baud, cfg.nos_uri);
    log_msg("umbilical: TC in :%d, TM out %s:%d", cfg.umb_tc_port, cfg.umb_host, cfg.umb_tm_port);
    log_msg("RF link:   in :%d, out %s:%d", cfg.rf_rx_port, cfg.rf_host, cfg.rf_tx_port);
    log_msg("torquers:  %s:%d; time: bus '%s' on %s", cfg.trq_host, cfg.trq_port, cfg.time_bus, cfg.time_uri);

    serial_fd = serial_open(cfg.serial_dev, cfg.baud);
    umb_sock  = udp_bind(cfg.umb_tc_port);
    rf_sock   = udp_bind(cfg.rf_rx_port);
    tx_sock   = socket(AF_INET, SOCK_DGRAM, 0);
    if (serial_fd < 0 || umb_sock < 0 || rf_sock < 0 || tx_sock < 0)
    {
        return 1;
    }

    init_workers();

    hub = NE_create_transport_hub(0);
    if (hub == NULL)
    {
        log_msg("failed to create NOS Engine transport hub");
        return 1;
    }

    if (strcmp(cfg.time_uri, "none") != 0)
    {
        time_bus = NE_create_bus(hub, cfg.time_bus, cfg.time_uri);
        if (time_bus == NULL)
        {
            log_msg("warning: cannot join time bus '%s' on %s; TIME frames disabled", cfg.time_bus, cfg.time_uri);
        }
    }

    for (i = 0; i < num_preopen; i++)
    {
        if (preopen[i] < 0 || preopen[i] >= HIL_NUM_BUSES)
        {
            log_msg("ignoring --uart %d (valid range 0-%d)", preopen[i], HIL_NUM_BUSES - 1);
            continue;
        }
        get_uart((uint8_t)preopen[i]);
    }

    hil_decoder_init(&dec);
    fds[0].fd     = serial_fd;
    fds[0].events = POLLIN;
    fds[1].fd     = umb_sock;
    fds[1].events = POLLIN;
    fds[2].fd     = rf_sock;
    fds[2].events = POLLIN;

    log_msg("running, waiting for MCU");
    while (keep_running)
    {
        int rc = poll(fds, 3, HIL_POLL_MS);
        if (rc < 0 && errno != EINTR)
        {
            log_msg("poll failed: %s", strerror(errno));
            break;
        }

        if (rc > 0)
        {
            if (fds[0].revents & (POLLERR | POLLNVAL))
            {
                log_msg("serial device error, exiting");
                break;
            }
            if (fds[0].revents & (POLLIN | POLLHUP))
            {
                read_serial(&dec);
            }
            if (fds[1].revents & POLLIN)
            {
                udp_to_mcu(umb_sock, HIL_CI_PKT);
            }
            if (fds[2].revents & POLLIN)
            {
                udp_to_mcu(rf_sock, HIL_RF_RX);
            }
        }

        poll_uarts();

        /* Only while the MCU is talking: writes to a serial device nobody reads can block */
        if (mcu_link_up && time(NULL) - last_time_sent >= HIL_TIME_PERIOD_S)
        {
            last_time_sent = time(NULL);
            send_time();
        }

        if (mcu_link_up && time(NULL) - last_mcu_frame > HIL_LINK_TIMEOUT_S)
        {
            mcu_link_up = 0;
            log_msg("MCU link down (no frames for %d s)", HIL_LINK_TIMEOUT_S);
        }
    }

    log_msg("shutting down");
    close_buses();
    if (time_bus != NULL)
    {
        NE_destroy_bus(&time_bus);
    }
    NE_destroy_transport_hub(&hub);
    close(serial_fd);
    close(umb_sock);
    close(rf_sock);
    close(tx_sock);
    return 0;
}
