#include <AP_uartTest/AP_uart_test.h>

extern const AP_HAL::HAL& hal;

void UARTTest::init(const AP_SerialManager& serial_manager) {
    // setup UARTx here

    // serial mappings according to: https://ardupilot.org/plane/docs/common-holybro-pixhawk6X.html
    //SERIAL0 -> USB
    //SERIAL1 -> UART7 (Telem1) RTS/CTS pins
    //SERIAL2 -> UART5 (Telem2) RTS/CTS pins
    //SERIAL3 -> USART1 (GPS1)
    //SERIAL4 -> UART8 (GPS2) 
    //SERIAL5 -> USART2 (Telem3) RTS/CTS pins
    //SERIAL6 -> UART4 (User)
    //SERIAL7 -> USART3 (Debug)
    //SERIAL8 -> USB 

    //setup UART
    uart_read = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Strain, 0);
    uart_write = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Strain, 1);
    
    //uart = hal.serial(6);
    //uart->set_blocking_writes(true);
    
    //uart->begin(baud);
    //uart->printf("init");
}

void UARTTest::write_uart() { // this is the function that main calls all the time
    // print counter to UARTx here
    //uart->printf("%s",&num);
    //uart->write(&num);
    //const ssize_t ret = uart->write(&counter, dataSize);

    //read all data
    bool res = read_data();

    //parse all data
    //parse_data();

    //write all data
    //for (uint8_t i = 0; i < 128; i++) {
    if (res) {
        for (uint8_t i = 0; i<available_bytes; i++) {
            uart_write->write(read_buffer[i]);
        }
        uart_write->write(available_bytes);
    }
    available_bytes = 0; // reset available bytes for next timeS

    //}
}

void UARTTest::parse_data() {

    for (uint8_t sensor = 0; sensor < NUM_SENSORSS; sensor++) {
        strain_data[sensor] = read_buffer[sensor+3]<<24 | read_buffer[sensor+2]<<16 |read_buffer[sensor+1]<<8 |read_buffer[sensor];
    }
}

/*bool UARTTest::read_data(void) {

    uint32_t nbytes = uart_read->available();

    if (nbytes != AP_SERIALMANAGER_STRAIN_BUFSIZE_RX) { //check that we have received all bytes from strain data
        for (uint8_t idx = 0; idx < nbytes; idx++) {
            read_buffer[idx] = uart_read->read();
        } 
        return true;
    }
    return false; 
} */

bool UARTTest::read_data(void) {

    available_bytes = uart_read->available();

    if (available_bytes != 0) { //check that we have received all bytes from strain data
        for (uint8_t idx = 0; idx < available_bytes; idx++) {
            read_buffer[idx] = uart_read->read();
        } 
        return true;
    }
    return false; 
} 

void UARTTest::save_data(void) {
    struct log_test pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STRAIN_MSG);
        time_us :   AP_HAL::micros64();
        value   :   1234;
    }
    logger.WriteBlock(&pkt, sizeof(pkt));
}