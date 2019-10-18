#include "serial.h"

Serial::Serial(uart_device_number_t channel, uint32_t baud_rate, uart_bitwidth_t data_width, uart_stopbit_t stopbit, uart_parity_t parity)
    : channel(channel), baud_rate(baud_rate), data_width(data_width), stopbit(stopbit), parity(parity)
{
    uart_init(channel);
    uart_config(channel, baud_rate, data_width, stopbit, parity);
}

int Serial::read(char* buffer, size_t size)
{
    return uart_receive_data(channel, buffer, size);
}

int Serial::write(const char *data, size_t length)
{
    return uart_send_data(channel, data, length);
}
