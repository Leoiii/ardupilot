#pragma once
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Logger/AP_Logger.h>
#define NUM_SENSORSS 32 // total number of strain sensors
#define BYTES_PER_SENSOR 4; // bytes per sensor (uint32)
#define READ_BUFFER_SIZE 128 // there are 32 sensors, each has 4 bytes of data

#define WRITE_UART_IDX 5 

//#define MASK 0xFF

class UARTTest 
{ 
public:
    UARTTest(){};

    void init(const AP_SerialManager& serial_manager);
    void write_uart();
    uint8_t counter = 0;

    char num = 47;

    static const uint32_t baud = 57600;
    AP_HAL::UARTDriver *uart_write = nullptr;
    AP_HAL::UARTDriver *uart_read = nullptr;

    uint8_t buf[4];

private:
    bool read_data(void); // read data from uart buffers
    void parse_data(void); // parse uart data into strain data
    uint8_t read_buffer[READ_BUFFER_SIZE];
    uint32_t strain_data[NUM_SENSORSS];   
    uint8_t available_bytes = 0;
    void save_data(void);

};

