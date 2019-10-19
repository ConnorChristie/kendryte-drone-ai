#include "multiwii.h"

namespace Msp
{

char* concat_arrays(char* array1, size_t size1, char* array2, size_t size2)
{
    char* result = (char*)malloc(size1 + size2);
    std::copy(array1, array1 + size1, result);
    std::copy(array2, array2 + size2, result + size1);

    return result;
}

uint8_t calc_checksum(MspCommand command, char* params, size_t param_size)
{
    uint8_t crc = param_size ^ command;

    for (unsigned int i = 0; i < param_size; i++)
    {
        crc ^= params[i];
    }

    return crc;
}

void print_bytes(char* data, size_t length)
{
    // Print out raw bytes
    for (unsigned int i = 0; i < length; i++)
        printk("%02X ", (unsigned char)data[i]);

    printk("\n");
}

constexpr int CLEAR_AMOUNT = 128;

void drain_read_buffer(Serial* serial)
{
    char* buf = new char[CLEAR_AMOUNT];
    auto drained_amount = serial->read(buf, CLEAR_AMOUNT);

    // LOGI("MSP", "Drained %d bytes from read buffer.", drained_amount);
    // print_bytes(buf, drained_amount);

    delete[] buf;

    // If we filled this buffer, there might be more to drain
    if (drained_amount == CLEAR_AMOUNT)
    {
        drain_read_buffer(serial);
    }
}

char* send_raw_command(Serial* serial, MspCommand command, char* param_data, uint8_t param_size)
{
    uint8_t send_param_size = param_data != NULL ? param_size : 0;

    MspRequest request = {
        .preamble = { '$', 'M' },
        .direction = '<',

        .size = send_param_size,
        .command = command,
    };

    char* result = (char*)& request;
    size_t result_size = sizeof(request);

    if (param_data != NULL)
    {
        // request + params
        result = concat_arrays(
            result, result_size,
            param_data, send_param_size
        );
        result_size += send_param_size;
    }

    uint8_t crc = calc_checksum(command, param_data, send_param_size);

    // + crc
    size_t crc_size = sizeof(crc);
    result = concat_arrays(
        result, result_size,
        (char*)& crc, crc_size
    );
    result_size += crc_size;

    // Write to serial port
    auto write_length = serial->write(result, result_size);

    // LOGI("MSP", "Sending:");
    // print_bytes(result, result_size);

    if (write_length != result_size)
    {
        LOGE("MSP", "Error writing %zu only wrote %d", result_size, write_length);
        print_bytes(result, result_size);
        return NULL;
    }

    // Wait 10ms to ensure we get full response
    // Might be able to use an interrupt but the largest threshold available is only 14 bytes...
    usleep(100 * 1000);

    // Size of request + params + checksum
    // If we are sending data, only an ack will be sent back (no data)
    size_t rcv_param_size = param_data == NULL ? param_size : 0;
    size_t rcv_size = sizeof(MspRequest) + rcv_param_size + sizeof(uint8_t);

    char* rcv_data = new char[rcv_size];
    auto read_length = serial->read(rcv_data, rcv_size);

    // Verify response
    // Verify checksum
    // Return params
    // $M> <size> <command> | <params> <checksum>

    MspRequest rcv_request;
    memcpy(&rcv_request, rcv_data, sizeof(MspRequest));

    // LOGI("MSP", "Recving:");
    // print_bytes(rcv_data, rcv_size);

    if (read_length != rcv_size)
    {
        LOGE("MSP", "Error reading %zu bytes, only read %d (response told us to read %u bytes)", rcv_size, read_length, rcv_request.size);
        print_bytes(rcv_data, rcv_size);
        drain_read_buffer(serial);
        return NULL;
    }

    // Verify direction is correct and command matches the one we requested
    if (rcv_request.direction != '>' || rcv_request.command != command)
    {
        LOGE("MSP", "Was expecting a response to our request but received different");
        print_bytes(rcv_data, rcv_size);
        drain_read_buffer(serial);
        return NULL;
    }

    if (rcv_request.size != rcv_param_size)
    {
        LOGE("MSP", "Was expecting a response size of %zu but received %u instead", rcv_param_size, rcv_request.size);
        print_bytes(rcv_data, rcv_size);
        drain_read_buffer(serial);
        return NULL;
    }

    char* rcv_params = new char[rcv_param_size];
    memcpy(rcv_params, rcv_data + sizeof(MspRequest), rcv_param_size);

    uint8_t calc_crc = calc_checksum(rcv_request.command, (char*)rcv_params, rcv_request.size);
    uint8_t actual_crc;

    memcpy(&actual_crc, rcv_data + sizeof(MspRequest) + rcv_param_size, sizeof(actual_crc));

    // Verify the checksum is correct
    if (calc_crc != actual_crc)
    {
        LOGE("MSP", "Bad checksum: calculated %u, recv %u", calc_crc, actual_crc);
        return NULL;
    }

    free(result);
    delete[] rcv_data;

    return rcv_params;
}

}
