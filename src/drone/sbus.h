#ifndef _SBUS_H_
#define _SBUS_H_

#include <stdlib.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include "atomic.h"
#include "uart.h"
#include "serial.h"
#include "ring_buffer.h"

#define SBUS_FRAME_SIZE  25
#define SBUS_FRAME_BEGIN 0x0F
#define SBUS_FRAME_END   0x00
#define SBUS_READ_BUFFER_SIZE (4 * SBUS_FRAME_SIZE)

typedef struct sbus_channels_s
{
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
    uint8_t flags;
} __attribute__((__packed__)) sbus_channels_t;

struct sbus_frame_s
{
    uint8_t syncByte;
    sbus_channels_t channels;
    uint8_t endByte;
} __attribute__ ((__packed__));

typedef union sbus_frame_u
{
    uint8_t bytes[SBUS_FRAME_SIZE];
    struct sbus_frame_s frame;
} sbus_frame_t;

struct sbus_buffer_s
{
    RingBuffer *ring_buffer;
    uint8_t read_buffer[SBUS_READ_BUFFER_SIZE];
};

namespace SBUS
{
    void init(uart_device_number_t device_number);
    int on_sbus_rcv(void *ctx);
    uint16_t raw_value_to_normal(const unsigned int raw_val);
};

#endif