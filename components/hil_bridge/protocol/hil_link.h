/*
** HIL Link - serial framing shared by the NOS3 HIL bridge and the MCU firmware
**
** Wire format (one frame):
**   COBS( type u8 | bus u8 | seq u8 | status u8 | addr u32 LE | payload[0..N] | crc16 LE ) 0x00
**
** The CRC is CRC-16/CCITT-FALSE over the header and payload. The trailing 0x00 byte delimits
** frames, and COBS guarantees it never appears inside one. This file has no OS, libc-I/O or
** NOS Engine dependencies so it compiles unchanged on a microcontroller.
*/
#ifndef _HIL_LINK_H_
#define _HIL_LINK_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Largest payload in one frame; override at compile time to save RAM on small MCUs */
#ifndef HIL_MAX_PAYLOAD
#define HIL_MAX_PAYLOAD 2048
#endif

#define HIL_HEADER_LEN 8
#define HIL_CRC_LEN    2
#define HIL_RAW_MAX    (HIL_HEADER_LEN + HIL_MAX_PAYLOAD + HIL_CRC_LEN)
/* COBS adds one byte per 254 plus one, and there is one delimiter byte */
#define HIL_ENCODED_MAX (HIL_RAW_MAX + (HIL_RAW_MAX / 254) + 2)

/*
** Frame types. "MCU->" frames are requests from the flight computer, "->MCU" frames come from
** the bridge. Every request carries a seq number that the bridge echoes in its response.
*/
typedef enum
{
    HIL_HEARTBEAT = 0x01, /* both ways: MCU sends, bridge echoes */
    HIL_LOG       = 0x02, /* MCU->  : payload is text, printed by the bridge */

    HIL_UART_TX   = 0x10, /* MCU->  : write payload to usart_<bus> */
    HIL_UART_RX   = 0x11, /* ->MCU  : bytes received on usart_<bus> (unsolicited) */
    HIL_UART_OPEN = 0x12, /* MCU->  : open usart_<bus> so RX starts flowing before first TX */

    HIL_I2C_TXN   = 0x20, /* MCU->  : payload = rxlen u16 LE | tx bytes; addr = 7-bit address */
    HIL_I2C_RSP   = 0x21, /* ->MCU  : payload = rx bytes, status set */

    HIL_SPI_TXN   = 0x30, /* MCU->  : payload = rxlen u16 LE | tx bytes; addr = chip select */
    HIL_SPI_RSP   = 0x31, /* ->MCU  : payload = rx bytes, status set */

    HIL_CAN_TXN   = 0x40, /* MCU->  : payload = rxlen u16 LE | tx bytes; addr = CAN identifier */
    HIL_CAN_RSP   = 0x41, /* ->MCU  : payload = rx bytes, status set */

    HIL_CI_PKT    = 0x50, /* ->MCU  : uplinked command packet (radio sim -> nos-fsw:5010) */
    HIL_TO_PKT    = 0x51, /* MCU->  : telemetry packet (-> radio-sim:5011) */
    HIL_RADIO_RX  = 0x52, /* ->MCU  : radio device traffic (radio sim -> nos-fsw:5015) */
    HIL_RADIO_TX  = 0x53  /* MCU->  : radio device command (-> radio-sim:5014) */
} hil_type_t;

typedef enum
{
    HIL_STATUS_OK        = 0,
    HIL_STATUS_BUS_ERROR = 1, /* NOS Engine transaction failed or bus could not be opened */
    HIL_STATUS_BAD_REQ   = 2, /* malformed request (bad bus number, length, ...) */
    HIL_STATUS_UNKNOWN   = 3  /* unknown frame type */
} hil_status_t;

typedef struct
{
    uint8_t  type;
    uint8_t  bus;
    uint8_t  seq;
    uint8_t  status;
    uint32_t addr;
    uint16_t len;
    uint8_t  payload[HIL_MAX_PAYLOAD];
} hil_frame_t;

typedef struct
{
    uint8_t buf[HIL_ENCODED_MAX];
    size_t  len;
    uint8_t overflow;
} hil_decoder_t;

typedef enum
{
    HIL_DECODE_NONE  = 0,  /* need more bytes */
    HIL_DECODE_FRAME = 1,  /* a valid frame was written to the output */
    HIL_DECODE_ERROR = -1  /* a frame ended but was corrupt (COBS, CRC, or too long) */
} hil_decode_result_t;

uint16_t hil_crc16(const uint8_t *data, size_t len);

/* Encode a frame (including the 0x00 delimiter) into out. Returns bytes written, 0 on error. */
size_t hil_encode(const hil_frame_t *frame, uint8_t *out, size_t out_cap);

void hil_decoder_init(hil_decoder_t *dec);

/* Feed one received byte. When it returns HIL_DECODE_FRAME, *frame holds the decoded frame. */
hil_decode_result_t hil_decoder_feed(hil_decoder_t *dec, uint8_t byte, hil_frame_t *frame);

/* Little-endian helpers for payload fields */
static inline uint16_t hil_get_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static inline void hil_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

#ifdef __cplusplus
}
#endif

#endif /* _HIL_LINK_H_ */
