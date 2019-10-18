#include "ring_buffer.h"

RingBuffer::RingBuffer(uint8_t _capacity)
{
    capacity = _capacity;
    buffer = malloc(_capacity * sizeof(char));
}

RingBuffer::~RingBuffer()
{
    free(buffer);
}

void RingBuffer::clear()
{
    head = 0;
    tail = 0;
}

bool RingBuffer::enqueue(const char &val)
{
    buffer[tail] = val;
    tail = (tail + 1) % capacity;

    if (tail == head)
    {
        head = (head + 1) % capacity;
        return true;
    }

    return false;
}

char RingBuffer::at(uint8_t index)
{
    return buffer[(head + index) % capacity];
}

void RingBuffer::getBuffer(char *out)
{
    uint8_t chunk1 = capacity - head;

    memcpy(out, buffer + head, chunk1);
    memcpy(out + chunk1, buffer, tail + 1);
}
