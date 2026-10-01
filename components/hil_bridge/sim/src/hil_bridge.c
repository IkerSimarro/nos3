/*
** NOS3 HIL Bridge
**
** Stands in for the cFS flight software container (hostname nos-fsw) so that a physical MCU,
** connected over a serial port, can act as the flight computer:
**
**   MCU <--hil_link frames--> hil_bridge --NOS Engine--> usart_N / i2c_N / spi_N / can_N --> device sims
**                                        --UDP-------->  radio-sim (CI 5010 in, TO 5011 out,
**                                                                   radio 5015 in, 5014 out)
**
** Bus handling mirrors fsw/apps/hwlib/sim/src (lib*.c, nos_link.c): buses are opened lazily on first
** use with the same bus names, node name and master address, so the device sims need no changes.
*/
#define _DEFAULT_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <netdb.h>
#include <poll.h>
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

typedef struct
{
    const char *serial_dev;
    int         baud;
    const char *nos_uri;
    const char *radio_host;
    int         ci_port;       /* listen: commands from radio sim */
    int         to_port;       /* send:   telemetry to radio sim */
    int         radio_rx_port; /* listen: radio device traffic from radio sim */
    int         radio_tx_port; /* send:   radio device commands to radio sim */
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

static hil_config_t cfg = {
    .serial_dev    = "/dev/ttyHIL0",
    .baud          = 921600,
    .nos_uri       = "tcp://nos-engine-server:12000",
    .radio_host    = "radio-sim",
    .ci_port       = 5010,
    .to_port       = 5011,
    .radio_rx_port = 5015,
    .radio_tx_port = 5014,
    .verbose       = 0,
};

static NE_TransportHub *hub = NULL;
static NE_Uart         *uart_dev[HIL_NUM_BUSES];
static NE_I2CHandle    *i2c_dev[HIL_NUM_BUSES];
static NE_SpiHandle    *spi_dev[HIL_NUM_BUSES];
static NE_CanHandle    *can_dev[HIL_NUM_BUSES];

static int serial_fd   = -1;
static int ci_sock     = -1;
static int radio_sock  = -1;
static int tx_sock     = -1;
static uint8_t bridge_seq = 0;

static hil_udp_dest_t to_dest;
static hil_udp_dest_t radio_dest;

/* Frame buffers are large; keep them off the stack */
static hil_frame_t rx_frame;
static hil_frame_t tx_frame;
static uint8_t     tx_wire[HIL_ENCODED_MAX];

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
            log_msg("serial write failed: %s", strerror(errno));
            return;
        }
        off += (size_t)w;
    }

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
static void udp_to_mcu(int fd, uint8_t type)
{
    static uint8_t buf[HIL_UDP_MAX];
    ssize_t        n = recv(fd, buf, sizeof(buf), 0);

    if (n <= 0)
    {
        return;
    }
    if ((size_t)n > HIL_MAX_PAYLOAD)
    {
        log_msg("dropping %zd-byte packet for MCU (max %d); raise HIL_MAX_PAYLOAD", n, HIL_MAX_PAYLOAD);
        return;
    }
    serial_send(type, 0, bridge_seq++, HIL_STATUS_OK, 0, buf, (size_t)n);
}

/*
** NOS Engine buses (lazy open, same as hwlib sim)
*/
static NE_Uart *get_uart(uint8_t bus)
{
    char name[16];

    if (bus >= HIL_NUM_BUSES)
    {
        return NULL;
    }
    if (uart_dev[bus] == NULL)
    {
        snprintf(name, sizeof(name), "usart_%u", bus);
        uart_dev[bus] = NE_uart_open3(hub, HIL_NOS_NODE_NAME, cfg.nos_uri, name, bus);
        if (uart_dev[bus] == NULL)
        {
            log_msg("failed to open %s on %s", name, cfg.nos_uri);
            return NULL;
        }
        NE_uart_set_queue_size(uart_dev[bus], HIL_USART_QUEUE_SIZE);
        log_msg("opened %s", name);
    }
    return uart_dev[bus];
}

static NE_I2CHandle *get_i2c(uint8_t bus)
{
    char name[16];

    if (bus >= HIL_NUM_BUSES)
    {
        return NULL;
    }
    if (i2c_dev[bus] == NULL)
    {
        snprintf(name, sizeof(name), "i2c_%u", bus);
        i2c_dev[bus] = NE_i2c_init_master3(hub, HIL_NOS_MASTER_ADDR, cfg.nos_uri, name);
        if (i2c_dev[bus] == NULL)
        {
            log_msg("failed to open %s on %s", name, cfg.nos_uri);
            return NULL;
        }
        log_msg("opened %s", name);
    }
    return i2c_dev[bus];
}

/* hwlib maps SPI (bus, cs) to NOS bus spi_<bus*10+cs> */
static NE_SpiHandle *get_spi(uint8_t bus, uint8_t cs)
{
    char     name[16];
    unsigned idx = (unsigned)bus * 10u + cs;

    if (cs >= 10 || idx >= HIL_NUM_BUSES)
    {
        return NULL;
    }
    if (spi_dev[idx] == NULL)
    {
        snprintf(name, sizeof(name), "spi_%u", idx);
        spi_dev[idx] = NE_spi_init_master3(hub, cfg.nos_uri, name);
        if (spi_dev[idx] == NULL)
        {
            log_msg("failed to open %s on %s", name, cfg.nos_uri);
            return NULL;
        }
        log_msg("opened %s", name);
    }
    return spi_dev[idx];
}

static NE_CanHandle *get_can(uint8_t bus)
{
    char name[16];

    if (bus >= HIL_NUM_BUSES)
    {
        return NULL;
    }
    if (can_dev[bus] == NULL)
    {
        snprintf(name, sizeof(name), "can_%u", bus);
        can_dev[bus] = NE_can_init_master3(hub, HIL_NOS_MASTER_ADDR, cfg.nos_uri, name);
        if (can_dev[bus] == NULL)
        {
            log_msg("failed to open %s on %s", name, cfg.nos_uri);
            return NULL;
        }
        log_msg("opened %s", name);
    }
    return can_dev[bus];
}

static void close_buses(void)
{
    int i;

    for (i = 0; i < HIL_NUM_BUSES; i++)
    {
        if (uart_dev[i]) NE_uart_close(&uart_dev[i]);
        if (i2c_dev[i])  NE_i2c_close(&i2c_dev[i]);
        if (spi_dev[i])  NE_spi_close(&spi_dev[i]);
        if (can_dev[i])  NE_can_close(&can_dev[i]);
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

static void handle_txn(const hil_frame_t *f)
{
    static uint8_t rx[HIL_MAX_PAYLOAD];
    const uint8_t *tx;
    size_t         txlen;
    size_t         rxlen;
    uint8_t        rsp_type = (uint8_t)(f->type + 1);
    uint8_t        status   = HIL_STATUS_BUS_ERROR;

    if (parse_txn(f, &tx, &txlen, &rxlen) != 0)
    {
        serial_send(rsp_type, f->bus, f->seq, HIL_STATUS_BAD_REQ, f->addr, NULL, 0);
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

    serial_send(rsp_type, f->bus, f->seq, status, f->addr, rx, (status == HIL_STATUS_OK) ? rxlen : 0);
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

        case HIL_UART_OPEN:
            if (get_uart(f->bus) == NULL)
            {
                log_msg("UART_OPEN: cannot open usart_%u", f->bus);
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
            handle_txn(f);
            break;

        case HIL_TO_PKT:
            udp_send(&to_dest, f->payload, f->len);
            break;

        case HIL_RADIO_TX:
            udp_send(&radio_dest, f->payload, f->len);
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
           "  -n, --nos-uri URI        NOS Engine server (default %s)\n"
           "  -r, --radio-host HOST    radio sim host (default %s)\n"
           "  -u, --uart N             open usart_N at startup (repeatable)\n"
           "      --ci-port P          UDP listen port for commands (default %d)\n"
           "      --to-port P          UDP port on radio host for telemetry (default %d)\n"
           "      --radio-rx-port P    UDP listen port for radio traffic (default %d)\n"
           "      --radio-tx-port P    UDP port on radio host for radio commands (default %d)\n"
           "  -v, --verbose            log every frame\n",
           prog, cfg.serial_dev, cfg.baud, cfg.nos_uri, cfg.radio_host, cfg.ci_port, cfg.to_port,
           cfg.radio_rx_port, cfg.radio_tx_port);
}

int main(int argc, char *argv[])
{
    static const struct option long_opts[] = {
        {"device", required_argument, NULL, 'd'},
        {"baud", required_argument, NULL, 'b'},
        {"nos-uri", required_argument, NULL, 'n'},
        {"radio-host", required_argument, NULL, 'r'},
        {"uart", required_argument, NULL, 'u'},
        {"ci-port", required_argument, NULL, 1},
        {"to-port", required_argument, NULL, 2},
        {"radio-rx-port", required_argument, NULL, 3},
        {"radio-tx-port", required_argument, NULL, 4},
        {"verbose", no_argument, NULL, 'v'},
        {"help", no_argument, NULL, 'h'},
        {NULL, 0, NULL, 0}};

    int           preopen[HIL_NUM_BUSES];
    int           num_preopen = 0;
    int           opt;
    int           i;
    hil_decoder_t dec;
    struct pollfd fds[3];
    struct sigaction sa;

    setvbuf(stdout, NULL, _IOLBF, 0);

    while ((opt = getopt_long(argc, argv, "d:b:n:r:u:vh", long_opts, NULL)) != -1)
    {
        switch (opt)
        {
            case 'd': cfg.serial_dev = optarg; break;
            case 'b': cfg.baud = atoi(optarg); break;
            case 'n': cfg.nos_uri = optarg; break;
            case 'r': cfg.radio_host = optarg; break;
            case 'u':
                if (num_preopen < HIL_NUM_BUSES)
                {
                    preopen[num_preopen++] = atoi(optarg);
                }
                break;
            case 1: cfg.ci_port = atoi(optarg); break;
            case 2: cfg.to_port = atoi(optarg); break;
            case 3: cfg.radio_rx_port = atoi(optarg); break;
            case 4: cfg.radio_tx_port = atoi(optarg); break;
            case 'v': cfg.verbose = 1; break;
            case 'h': usage(argv[0]); return 0;
            default: usage(argv[0]); return 1;
        }
    }

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = handle_signal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    to_dest.name    = cfg.radio_host;
    to_dest.port    = cfg.to_port;
    radio_dest.name = cfg.radio_host;
    radio_dest.port = cfg.radio_tx_port;

    log_msg("serial %s @ %d, NOS Engine %s, radio %s (CI in %d, TO out %d, radio in %d, radio out %d)",
            cfg.serial_dev, cfg.baud, cfg.nos_uri, cfg.radio_host, cfg.ci_port, cfg.to_port,
            cfg.radio_rx_port, cfg.radio_tx_port);

    serial_fd  = serial_open(cfg.serial_dev, cfg.baud);
    ci_sock    = udp_bind(cfg.ci_port);
    radio_sock = udp_bind(cfg.radio_rx_port);
    tx_sock    = socket(AF_INET, SOCK_DGRAM, 0);
    if (serial_fd < 0 || ci_sock < 0 || radio_sock < 0 || tx_sock < 0)
    {
        return 1;
    }

    hub = NE_create_transport_hub(0);
    if (hub == NULL)
    {
        log_msg("failed to create NOS Engine transport hub");
        return 1;
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
    fds[1].fd     = ci_sock;
    fds[1].events = POLLIN;
    fds[2].fd     = radio_sock;
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
                udp_to_mcu(ci_sock, HIL_CI_PKT);
            }
            if (fds[2].revents & POLLIN)
            {
                udp_to_mcu(radio_sock, HIL_RADIO_RX);
            }
        }

        poll_uarts();

        if (mcu_link_up && time(NULL) - last_mcu_frame > HIL_LINK_TIMEOUT_S)
        {
            mcu_link_up = 0;
            log_msg("MCU link down (no frames for %d s)", HIL_LINK_TIMEOUT_S);
        }
    }

    log_msg("shutting down");
    close_buses();
    NE_destroy_transport_hub(&hub);
    close(serial_fd);
    close(ci_sock);
    close(radio_sock);
    close(tx_sock);
    return 0;
}
