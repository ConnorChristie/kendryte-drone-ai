#ifndef _RING_BUFFER_H_
#define _RING_BUFFER_H_

#include <stdlib.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>

class RingBuffer
{
private:
    uint8_t capacity = 0;
    uint8_t head = 0;
    uint8_t tail = 0;

    char *buffer;

public:
    RingBuffer(uint8_t capacity);
    ~RingBuffer();

    void clear();

    bool enqueue(const char &val);
    char at(uint8_t index);

    void getBuffer(char* out);
};

#endif