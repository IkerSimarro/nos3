/*
** HIL Link - serial framing shared by the NOS3 HIL bridge and the MCU firmware
** See hil_link.h for the wire format.
*/
#include "hil_link.h"

#include <string.h>

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor */
uint16_t hil_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    size_t   i;
    int      b;

    for (i = 0; i < len; i++)
    {
        crc ^= (uint16_t)(data[i] << 8);
        for (b = 0; b < 8; b++)
        {
            if (crc & 0x8000)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            }
            else
            {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* COBS-encode in[0..len) into out; returns encoded length (without delimiter) or 0 if out is too small */
static size_t hil_cobs_encode(const uint8_t *in, size_t len, uint8_t *out, size_t out_cap)
{
    size_t  read_idx  = 0;
    size_t  write_idx = 1;
    size_t  code_idx  = 0;
    uint8_t code      = 1;

    if (out_cap == 0)
    {
        return 0;
    }

    while (read_idx < len)
    {
        if (write_idx >= out_cap)
        {
            return 0;
        }

        if (in[read_idx] == 0)
        {
            out[code_idx] = code;
            code          = 1;
            code_idx      = write_idx++;
        }
        else
        {
            out[write_idx++] = in[read_idx];
            code++;
            if (code == 0xFF)
            {
                out[code_idx] = code;
                code          = 1;
                code_idx      = write_idx++;
            }
        }
        read_idx++;
    }

    if (code_idx >= out_cap)
    {
        return 0;
    }
    out[code_idx] = code;
    return write_idx;
}

/* COBS-decode in[0..len) into out; returns decoded length or 0 on malformed input */
static size_t hil_cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t out_cap)
{
    size_t  read_idx  = 0;
    size_t  write_idx = 0;
    uint8_t code;
    uint8_t i;

    while (read_idx < len)
    {
        code = in[read_idx];
        if (code == 0 || read_idx + code > len)
        {
            return 0;
        }
        read_idx++;

        for (i = 1; i < code; i++)
        {
            if (read_idx >= len || write_idx >= out_cap)
            {
                return 0;
            }
            out[write_idx++] = in[read_idx++];
        }

        /* A code of 0xFF means "254 data bytes, no implied zero"; the final group also has none */
        if (code != 0xFF && read_idx < len)
        {
            if (write_idx >= out_cap)
            {
                return 0;
            }
            out[write_idx++] = 0;
        }
    }
    return write_idx;
}

size_t hil_encode(const hil_frame_t *frame, uint8_t *out, size_t out_cap)
{
    uint8_t  raw[HIL_RAW_MAX];
    size_t   raw_len;
    size_t   enc_len;
    uint16_t crc;

    if (frame == NULL || out == NULL || frame->len > HIL_MAX_PAYLOAD)
    {
        return 0;
    }

    raw[0] = frame->type;
    raw[1] = frame->bus;
    raw[2] = frame->seq;
    raw[3] = frame->status;
    raw[4] = (uint8_t)(frame->addr & 0xFF);
    raw[5] = (uint8_t)((frame->addr >> 8) & 0xFF);
    raw[6] = (uint8_t)((frame->addr >> 16) & 0xFF);
    raw[7] = (uint8_t)((frame->addr >> 24) & 0xFF);
    memcpy(&raw[HIL_HEADER_LEN], frame->payload, frame->len);
    raw_len = HIL_HEADER_LEN + frame->len;

    crc = hil_crc16(raw, raw_len);
    hil_put_u16(&raw[raw_len], crc);
    raw_len += HIL_CRC_LEN;

    /* Reserve one byte for the delimiter */
    if (out_cap < 2)
    {
        return 0;
    }
    enc_len = hil_cobs_encode(raw, raw_len, out, out_cap - 1);
    if (enc_len == 0)
    {
        return 0;
    }
    out[enc_len] = 0x00;
    return enc_len + 1;
}

void hil_decoder_init(hil_decoder_t *dec)
{
    dec->len      = 0;
    dec->overflow = 0;
}

hil_decode_result_t hil_decoder_feed(hil_decoder_t *dec, uint8_t byte, hil_frame_t *frame)
{
    uint8_t  raw[HIL_RAW_MAX];
    size_t   raw_len;
    uint16_t crc;
    size_t   enc_len;
    uint8_t  overflow;

    if (byte != 0x00)
    {
        if (dec->len < sizeof(dec->buf))
        {
            dec->buf[dec->len++] = byte;
        }
        else
        {
            dec->overflow = 1;
        }
        return HIL_DECODE_NONE;
    }

    /* Delimiter: a frame (or noise) has ended */
    enc_len  = dec->len;
    overflow = dec->overflow;
    hil_decoder_init(dec);

    if (enc_len == 0)
    {
        return HIL_DECODE_NONE; /* back-to-back delimiters are allowed as idle fill */
    }
    if (overflow)
    {
        return HIL_DECODE_ERROR;
    }

    raw_len = hil_cobs_decode(dec->buf, enc_len, raw, sizeof(raw));
    if (raw_len < HIL_HEADER_LEN + HIL_CRC_LEN)
    {
        return HIL_DECODE_ERROR;
    }

    crc = hil_crc16(raw, raw_len - HIL_CRC_LEN);
    if (crc != hil_get_u16(&raw[raw_len - HIL_CRC_LEN]))
    {
        return HIL_DECODE_ERROR;
    }

    frame->type   = raw[0];
    frame->bus    = raw[1];
    frame->seq    = raw[2];
    frame->status = raw[3];
    frame->addr   = (uint32_t)raw[4] | ((uint32_t)raw[5] << 8) | ((uint32_t)raw[6] << 16) | ((uint32_t)raw[7] << 24);
    frame->len    = (uint16_t)(raw_len - HIL_HEADER_LEN - HIL_CRC_LEN);
    memcpy(frame->payload, &raw[HIL_HEADER_LEN], frame->len);
    return HIL_DECODE_FRAME;
}
