#pragma once
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Logger/LogStructure.h>
#define NUM_SENSORSS 32 // total number of strain sensors
#define BYTES_PER_SENSOR 3 // bytes per sensor (uint32)
#define READ_BUFFER_SIZE 96 // there are 32 sensors, each has 3 bytes of data

#define WRITE_UART_IDX 5 


//#define MASK 0xFF

class UARTTest 
{ 
public:
    UARTTest(){};

    void init(const AP_SerialManager& serial_manager);
    void write_uart();
    void log_strain(uint32_t *strain_array);
    void log_empty(uint16_t val);

    //add some flag for crc here

    static const uint32_t baud = 57600;
    AP_HAL::UARTDriver *uart_write = nullptr;
    AP_HAL::UARTDriver *uart_read = nullptr;

    uint8_t buf[4];

private:
    uint16_t read_data(void); // read data from uart buffers
    void parse_data(void); // parse uart data into strain data
    uint8_t read_buffer[READ_BUFFER_SIZE];
    uint8_t *read_bufferp = &read_buffer[0];
    uint32_t strain_data[NUM_SENSORSS];   
    uint8_t available_bytes;
    uint16_t counter;

};

