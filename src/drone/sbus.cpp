#include "sbus.h"

SBUS::SBUS(uart_device_number_t device_number)
{
    serial = new Serial(device_number, 100000, UART_BITWIDTH_8BIT, UART_STOP_2, UART_PARITY_EVEN);
    ring_buffer = new RingBuffer(SBUS_FRAME_SIZE);

    uart_receive_data_dma_irq(device_number, DMAC_CHANNEL3, read_buffer, SBUS_READ_BUFFER_SIZE, on_sbus_rcv, this, 1);
}

int SBUS::on_data()
{
    for (int i = 0; i < SBUS_READ_BUFFER_SIZE; i++)
    {
        if (ring_buffer->enqueue(read_buffer[i]))
        {
            // Check if head and tail make for a valid frame
            if (ring_buffer->at(0) == SBUS_FRAME_BEGIN && ring_buffer->at(SBUS_FRAME_SIZE - 1) == SBUS_FRAME_END)
            {
                sbus_frame_t frame;

                ring_buffer->getBuffer(frame.bytes);
                ring_buffer->clear();

                printf("Read: %u, %u, %u, %u, %u, %u, %u\n",
                    sbus_raw_value_to_normal(frame.frame.channels.chan0),
                    sbus_raw_value_to_normal(frame.frame.channels.chan1),
                    sbus_raw_value_to_normal(frame.frame.channels.chan2),
                    sbus_raw_value_to_normal(frame.frame.channels.chan3),
                    sbus_raw_value_to_normal(frame.frame.channels.chan4),
                    sbus_raw_value_to_normal(frame.frame.channels.chan5),
                    sbus_raw_value_to_normal(frame.frame.channels.chan6));
            }
        }
    }

    uart_receive_data_dma_irq(UART_DEVICE_2, DMAC_CHANNEL3, read_buffer, SBUS_READ_BUFFER_SIZE, on_sbus_rcv, this, 1);
    return 0;
}

static int on_sbus_rcv(void *ctx)
{
    return static_cast<SBUS*>(ctx)->on_data();
}

static uint16_t sbus_raw_value_to_normal(const unsigned int raw_val)
{
    return (5 * raw_val / 8) + 880;
}
