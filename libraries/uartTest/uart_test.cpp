#include <uart_test.h>

extern const AP_HAL::HAL& hal;

void UARTTest::init(void) {
    // setup UARTx here

    counter = 0; // initialize counter variable

    //setup UART
    uart = hal.serial(2);

    uart->begin(baud);
}

void UARTTest::write_uart() {
    // print counter to UARTx here

    const ssize_t ret = uart->write(&counter, dataSize);
    counter = ret;
}