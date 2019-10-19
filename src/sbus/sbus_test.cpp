#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bsp.h"
#include "dvp.h"
#include "fpioa.h"
#include "gpiohs.h"
#include "plic.h"
#include "sysctl.h"
#include "uart.h"
#include "uarths.h"

#define PLL0_OUTPUT_FREQ 800000000UL
#define PLL1_OUTPUT_FREQ 400000000UL

static uart_device_number_t SBUS_DEVICE_NUMBER = UART_DEVICE_2;

#define SBUS_FRAME_SIZE  25
#define SBUS_FRAME_BEGIN 0x0F
#define SBUS_FRAME_END   0x00
#define SBUS_READ_BUFFER_SIZE (4 * SBUS_FRAME_SIZE)

#define SINGLE_READ_SIZE 25
#define READ_BUFFER_SIZE (SINGLE_READ_SIZE * 4)

uint32_t recv_buf[READ_BUFFER_SIZE];

// char recv_buf[SBUS_READ_BUFFER_SIZE];
volatile bool recv_flag = false;
volatile uint8_t bytes_read = 0;

static void print_bytes(char* data, size_t length)
{
    // Print out raw bytes
    for (unsigned int i = 0; i < length; i++)
    {
        // char x = (data[i] & 0x0F) << 4 | (data[i] & 0xF0) >> 4;
        printf("%02X ", (unsigned char)data[i]);
    }

    // printf("\n");
}

static int on_sbus_rcv(void *ctx)
{
    uint32_t *v_buf = (uint32_t*)ctx;

    // for (int i = 0; i < SINGLE_READ_SIZE; i++)
    // {
    //     auto b = (uint8_t)(v_buf[i] & 0xff);

    //     bytes_read++;

    //     if (b == SBUS_FRAME_BEGIN && !recv_flag)
    //     {
    //         recv_flag = true;
    //     }
    //     else if (b == SBUS_FRAME_END && recv_flag)
    //     {
    //         if (bytes_read >= SBUS_FRAME_SIZE)
    //         {
    //             char tmp[SBUS_FRAME_SIZE];
    //             // char *tmp = (char*)malloc(SBUS_FRAME_SIZE);

    //             auto current_pos = v_buf + i;
    //             auto head = current_pos - bytes_read;
    //             if (head < recv_buf)
    //             {
    //                 auto offset = recv_buf - head;
    //                 head = (recv_buf + SBUS_READ_BUFFER_SIZE) - offset;

    //                 for (auto n = 0; n < offset; n++)
    //                     tmp[n] = (uint8_t)(*(head + n) & 0xff);

    //                 for (auto n = 0; n < current_pos - recv_buf; n++)
    //                     tmp[n + offset] = (uint8_t)(*(recv_buf + n) & 0xff);
    //             }
    //             else
    //             {
    //                 for (auto n = 0; n < bytes_read; n++)
    //                     tmp[n] = (uint8_t)(*(head + n) & 0xff);
    //             }

    //             // printf("dest_tmp: %p\n", tmp);
    //             // free(tmp);

    //             // print_bytes(tmp, bytes_read);
    //         }

    //         recv_flag = false;
    //         bytes_read = 0;
    //     }
    // }

    // uint32_t *v_dest = v_buf + SINGLE_READ_SIZE;
    // if (v_dest >= recv_buf + READ_BUFFER_SIZE)
    //     v_dest = recv_buf;

    uart_data_t data = {
        .rx_channel = DMAC_CHANNEL3,
        .rx_buf = recv_buf,
        .rx_len = SINGLE_READ_SIZE,
        .transfer_mode = UART_RECEIVE
    };

    plic_interrupt_t irq = {
        .callback = on_sbus_rcv,
        .ctx = recv_buf,
        .priority = 10
    };

    printk("dest: %p\n", recv_buf);

    uart_handle_data_dma(SBUS_DEVICE_NUMBER, data, &irq);
    return 0;
}

int main(void)
{
    /* Set CPU and dvp clk */
    // sysctl_pll_set_freq(SYSCTL_PLL0, PLL0_OUTPUT_FREQ);
    // sysctl_pll_set_freq(SYSCTL_PLL1, PLL1_OUTPUT_FREQ);
    // sysctl_clock_enable(SYSCTL_CLOCK_AI);


    fpioa_set_function(9, FUNC_UART2_RX);
    // fpioa_set_function(10, FUNC_UART2_TX);

    // fpioa_io_config_t cfg;
    // fpioa_get_io(9, &cfg);

    // cfg.di_inv = 1;
    // fpioa_set_io(9, &cfg);


    // uarths_init();
    plic_init();

    /* enable global interrupt */
    // sysctl_enable_irq();

    uart_data_t v_rx_data = {
        .rx_channel = DMAC_CHANNEL3,
        .rx_buf = recv_buf,
        .rx_len = SINGLE_READ_SIZE,
        .transfer_mode = UART_RECEIVE
    };

    plic_interrupt_t v_rx_irq = {
        .callback = on_sbus_rcv,
        .ctx = recv_buf,
        .priority = 10
    };

    printk("Starting handler\n");

    uart_init(SBUS_DEVICE_NUMBER);
    uart_config(SBUS_DEVICE_NUMBER, 100000, UART_BITWIDTH_8BIT, UART_STOP_2, UART_PARITY_EVEN);
    uart_handle_data_dma(SBUS_DEVICE_NUMBER, v_rx_data, &v_rx_irq);

    while (1)
    {
        printk("hi\n");
    }
}