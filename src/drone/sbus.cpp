#include "sbus.h"

namespace SBUS
{

void init(uart_device_number_t device_number)
{
    sbus_buffer_s buffer;
    buffer.ring_buffer = new RingBuffer(SBUS_FRAME_SIZE);

    uart_init(device_number);
    uart_config(device_number, 100000, UART_BITWIDTH_8BIT, UART_STOP_2, UART_PARITY_EVEN);
    uart_receive_data_dma_irq(device_number, DMAC_CHANNEL3, buffer.read_buffer, SBUS_READ_BUFFER_SIZE, on_sbus_rcv, &buffer, 1);
}

int on_sbus_rcv(void *ctx)
{
    sbus_buffer_s *buffer = (sbus_buffer_s*)ctx;

    printk("inside here\n");

    for (int i = 0; i < SBUS_READ_BUFFER_SIZE; i++)
    {
        if (buffer->ring_buffer->enqueue(buffer->read_buffer[i]))
        {
            // Check if head and tail make for a valid frame
            if (buffer->ring_buffer->at(0) == SBUS_FRAME_BEGIN && buffer->ring_buffer->at(SBUS_FRAME_SIZE - 1) == SBUS_FRAME_END)
            {
                sbus_frame_t frame;

                buffer->ring_buffer->getBuffer(frame.bytes);
                buffer->ring_buffer->clear();
            }
        }
    }

    uart_receive_data_dma_irq(UART_DEVICE_2, DMAC_CHANNEL3, buffer->read_buffer, SBUS_READ_BUFFER_SIZE, on_sbus_rcv, ctx, 1);
    return 0;
}

uint16_t raw_value_to_normal(const unsigned int raw_val)
{
    return (5 * raw_val / 8) + 880;
}

};
