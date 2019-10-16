#ifndef _SERIAL_H_
#define _SERIAL_H_

#include "uart.h"
#include "fpioa.h"
#include "syslog.h"

class serial
{
public:
    serial(uart_device_number_t channel, uint32_t baud_rate, uart_bitwidth_t data_width, uart_stopbit_t stopbit, uart_parity_t parity);

    void init();
    int read(char* buf, size_t size);
    int write(const char *data, size_t length);

private:
    uart_device_number_t channel;
    uint32_t baud_rate;
    uart_bitwidth_t data_width;
    uart_stopbit_t stopbit;
    uart_parity_t parity;
};

#endif
