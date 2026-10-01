/*
** Example MCU-side use of hil_link: how flight software on the microcontroller talks to NOS3
** device sims and the ground through the HIL bridge.
**
** Port it by implementing the three platform hooks below for your board (USB CDC or UART
** driver + a millisecond tick), then call hil_example_step() from your main loop.
** Add ../protocol/hil_link.c to the firmware build; define HIL_MAX_PAYLOAD smaller (e.g. 512)
** if RAM is tight. Payloads larger than that are dropped by the bridge with a log message.
*/
#include <string.h>

#include "hil_link.h"

/* ---- Platform hooks: implement these for your board ---- */
extern void     platform_serial_write(const uint8_t *data, size_t len);
extern int      platform_serial_read_byte(uint8_t *byte); /* 1 if a byte was read, 0 if none waiting */
extern uint32_t platform_millis(void);

/* ---- Application hooks: your flight software ---- */
extern void fsw_handle_command(const uint8_t *pkt, size_t len);           /* uplinked CCSDS command */
extern void fsw_handle_uart_rx(uint8_t bus, const uint8_t *data, size_t len); /* bytes from a device sim */

static hil_decoder_t decoder;
static hil_frame_t   frame;    /* last received frame */
static hil_frame_t   tx_frame; /* separate so handlers may send while handling a received frame */
static uint8_t       wire[HIL_ENCODED_MAX];
static uint8_t       next_seq;

static void hil_send(hil_frame_t *f)
{
    size_t n = hil_encode(f, wire, sizeof(wire));
    if (n > 0)
    {
        platform_serial_write(wire, n);
    }
}

/*
** Route frames the bridge sends without being asked. Handlers run from inside hil_poll(), so they
** may send (telemetry etc.) but should not call hil_i2c_transaction(); queue the work instead.
*/
static void hil_dispatch_async(const hil_frame_t *f)
{
    switch (f->type)
    {
        case HIL_CI_PKT:
            fsw_handle_command(f->payload, f->len);
            break;
        case HIL_UART_RX:
            fsw_handle_uart_rx(f->bus, f->payload, f->len);
            break;
        default:
            break;
    }
}

/* Process received bytes; returns 1 and fills *out if a frame of rsp_type with seq arrived */
static int hil_poll(uint8_t rsp_type, uint8_t seq, hil_frame_t *out)
{
    uint8_t byte;

    while (platform_serial_read_byte(&byte))
    {
        if (hil_decoder_feed(&decoder, byte, &frame) != HIL_DECODE_FRAME)
        {
            continue;
        }
        if (out != NULL && frame.type == rsp_type && frame.seq == seq)
        {
            memcpy(out, &frame, sizeof(frame));
            return 1;
        }
        hil_dispatch_async(&frame);
    }
    return 0;
}

/*
** I2C transaction through the bridge: the equivalent of hwlib's i2c_master_transaction().
** Returns 0 on success. Other frames that arrive while waiting are still dispatched.
*/
int hil_i2c_transaction(uint8_t bus, uint8_t addr, const uint8_t *tx, size_t txlen,
                        uint8_t *rx, size_t rxlen, uint32_t timeout_ms)
{
    static hil_frame_t rsp;
    uint8_t            seq = next_seq++;
    uint32_t           start;

    if (txlen + 2 > HIL_MAX_PAYLOAD || rxlen > HIL_MAX_PAYLOAD)
    {
        return -1;
    }

    tx_frame.type   = HIL_I2C_TXN;
    tx_frame.bus    = bus;
    tx_frame.seq    = seq;
    tx_frame.status = 0;
    tx_frame.addr   = addr;
    hil_put_u16(tx_frame.payload, (uint16_t)rxlen);
    memcpy(tx_frame.payload + 2, tx, txlen);
    tx_frame.len = (uint16_t)(txlen + 2);
    hil_send(&tx_frame);

    start = platform_millis();
    while ((uint32_t)(platform_millis() - start) < timeout_ms)
    {
        if (hil_poll(HIL_I2C_RSP, seq, &rsp))
        {
            if (rsp.status != HIL_STATUS_OK || rsp.len != rxlen)
            {
                return -1;
            }
            memcpy(rx, rsp.payload, rxlen);
            return 0;
        }
    }
    return -1; /* timeout */
}

/* Write bytes to a device sim's USART (replies arrive via fsw_handle_uart_rx) */
void hil_uart_write(uint8_t bus, const uint8_t *data, size_t len)
{
    if (len > HIL_MAX_PAYLOAD)
    {
        return;
    }
    tx_frame.type   = HIL_UART_TX;
    tx_frame.bus    = bus;
    tx_frame.seq    = next_seq++;
    tx_frame.status = 0;
    tx_frame.addr   = 0;
    tx_frame.len    = (uint16_t)len;
    memcpy(tx_frame.payload, data, len);
    hil_send(&tx_frame);
}

/* Downlink a CCSDS telemetry packet (bridge forwards it to the radio sim -> ground) */
void hil_send_telemetry(const uint8_t *pkt, size_t len)
{
    if (len > HIL_MAX_PAYLOAD)
    {
        return;
    }
    tx_frame.type   = HIL_TO_PKT;
    tx_frame.bus    = 0;
    tx_frame.seq    = next_seq++;
    tx_frame.status = 0;
    tx_frame.addr   = 0;
    tx_frame.len    = (uint16_t)len;
    memcpy(tx_frame.payload, pkt, len);
    hil_send(&tx_frame);
}

/* ---- Example: poll the NOS3 generic EPS over i2c_1 @ 0x2B like the cFS generic_eps app ---- */
#define EPS_I2C_BUS  1
#define EPS_I2C_ADDR 0x2B
#define EPS_HK_LEN   64 /* sizeof(GENERIC_EPS_Device_HK_tlm_t) */

static uint8_t eps_crc8(const uint8_t *d, size_t len)
{
    uint8_t crc = 0xFF;
    size_t  i;
    int     b;
    for (i = 0; i < len; i++)
    {
        crc ^= d[i];
        for (b = 0; b < 8; b++)
        {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* Reads EPS housekeeping; returns battery voltage raw value or -1 on error */
int32_t hil_example_read_battery(void)
{
    uint8_t cmd[3] = {0x70, 0x00, 0};
    uint8_t hk[EPS_HK_LEN + 1];

    cmd[2] = eps_crc8(cmd, 2);
    if (hil_i2c_transaction(EPS_I2C_BUS, EPS_I2C_ADDR, cmd, sizeof(cmd), hk, sizeof(hk), 100) != 0)
    {
        return -1;
    }
    if (eps_crc8(hk, EPS_HK_LEN) != hk[EPS_HK_LEN])
    {
        return -1;
    }
    return (int32_t)((hk[0] << 8) | hk[1]);
}

void hil_example_init(void)
{
    hil_decoder_init(&decoder);
    next_seq = 0;
}

/* Call from the main loop: services uplinks/UART data and sends a heartbeat once a second */
void hil_example_step(void)
{
    static uint32_t last_hb;

    hil_poll(0, 0, NULL);

    if ((uint32_t)(platform_millis() - last_hb) >= 1000)
    {
        last_hb      = platform_millis();
        tx_frame.type   = HIL_HEARTBEAT;
        tx_frame.bus    = 0;
        tx_frame.seq    = next_seq++;
        tx_frame.status = 0;
        tx_frame.addr   = 0;
        tx_frame.len    = 0;
        hil_send(&tx_frame);
    }
}
