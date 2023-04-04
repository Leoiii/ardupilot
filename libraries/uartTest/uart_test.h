#include <AP_HAL/AP_HAL.h>

class UARTTest 
{ 
public:
    void init();
    void write_uart();

    uint8_t counter;
    static const uint8_t dataSize = sizeof(counter);

    static const uint32_t baud = 57600;
    AP_HAL::UARTDriver *uart = nullptr;

};


