/*
** Host-side unit test for hil_link.c
** Build and run: cc -Wall -Wextra -I. test_hil_link.c hil_link.c -o test_hil_link && ./test_hil_link
*/
#include "hil_link.h"

#include <stdio.h>
#include <string.h>

static int failures = 0;

#define CHECK(cond)                                                    \
    do                                                                 \
    {                                                                  \
        if (!(cond))                                                   \
        {                                                              \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
            failures++;                                                \
        }                                                              \
    } while (0)

static hil_frame_t in_frame;
static hil_frame_t out_frame;
static uint8_t     wire[HIL_ENCODED_MAX];

/* Encode in_frame, feed it back byte by byte, and check the result matches */
static void roundtrip(size_t payload_len, uint8_t fill_mode)
{
    hil_decoder_t dec;
    size_t        n;
    size_t        i;
    int           frames = 0;

    memset(&in_frame, 0, sizeof(in_frame));
    in_frame.type   = HIL_I2C_TXN;
    in_frame.bus    = 1;
    in_frame.seq    = 42;
    in_frame.status = 0;
    in_frame.addr   = 0x1234002B;
    in_frame.len    = (uint16_t)payload_len;
    for (i = 0; i < payload_len; i++)
    {
        /* mode 0: all zeros, mode 1: no zeros, mode 2: mixed */
        in_frame.payload[i] = (fill_mode == 0) ? 0 : (fill_mode == 1) ? (uint8_t)(1 + (i % 255)) : (uint8_t)(i * 7);
    }

    n = hil_encode(&in_frame, wire, sizeof(wire));
    CHECK(n > 0);
    CHECK(wire[n - 1] == 0x00);
    for (i = 0; i + 1 < n; i++)
    {
        CHECK(wire[i] != 0x00);
    }

    hil_decoder_init(&dec);
    for (i = 0; i < n; i++)
    {
        hil_decode_result_t r = hil_decoder_feed(&dec, wire[i], &out_frame);
        CHECK(r != HIL_DECODE_ERROR);
        if (r == HIL_DECODE_FRAME)
        {
            frames++;
        }
    }
    CHECK(frames == 1);
    CHECK(out_frame.type == in_frame.type);
    CHECK(out_frame.bus == in_frame.bus);
    CHECK(out_frame.seq == in_frame.seq);
    CHECK(out_frame.addr == in_frame.addr);
    CHECK(out_frame.len == in_frame.len);
    CHECK(memcmp(out_frame.payload, in_frame.payload, payload_len) == 0);
}

static void test_corruption(void)
{
    hil_decoder_t dec;
    size_t        n;
    size_t        i;
    int           errors = 0;
    int           frames = 0;

    memset(&in_frame, 0, sizeof(in_frame));
    in_frame.type = HIL_TO_PKT;
    in_frame.len  = 20;
    memset(in_frame.payload, 0xAA, 20);
    n = hil_encode(&in_frame, wire, sizeof(wire));

    wire[n - 6] ^= 0x40; /* flip a bit in the payload (0xAA -> 0xEA, never a delimiter) */
    hil_decoder_init(&dec);
    for (i = 0; i < n; i++)
    {
        hil_decode_result_t r = hil_decoder_feed(&dec, wire[i], &out_frame);
        errors += (r == HIL_DECODE_ERROR);
        frames += (r == HIL_DECODE_FRAME);
    }
    CHECK(errors == 1);
    CHECK(frames == 0);

    /* The decoder must resynchronise on the next good frame */
    wire[n - 6] ^= 0x40;
    for (i = 0; i < n; i++)
    {
        frames += (hil_decoder_feed(&dec, wire[i], &out_frame) == HIL_DECODE_FRAME);
    }
    CHECK(frames == 1);
}

static void test_known_crc(void)
{
    /* CRC-16/CCITT-FALSE check value */
    CHECK(hil_crc16((const uint8_t *)"123456789", 9) == 0x29B1);
}

int main(void)
{
    size_t lens[] = {0, 1, 3, 245, 246, 253, 254, 255, 508, 1000, HIL_MAX_PAYLOAD};
    size_t i;
    uint8_t mode;

    test_known_crc();
    for (i = 0; i < sizeof(lens) / sizeof(lens[0]); i++)
    {
        for (mode = 0; mode < 3; mode++)
        {
            roundtrip(lens[i], mode);
        }
    }
    test_corruption();

    if (failures)
    {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    printf("hil_link: all tests passed\n");
    return 0;
}
