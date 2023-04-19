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
    parse_data();

    uart_write->write(available_bytes);

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
    available_bytes = 0; // reset available bytes for next time

    crc_crc16_ibm

    Log_Strain(strain_data);

    //}
}

void UARTTest::parse_data() {

    for (uint8_t sensor = 0; sensor < NUM_SENSORSS; sensor++) {
        strain_data[sensor] = read_buffer[(sensor*BYTES_PER_SENSOR)+2]<<16 | 
                              read_buffer[(sensor*BYTES_PER_SENSOR)+1]<<8 | 
                              read_buffer[sensor*BYTES_PER_SENSOR];
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
    uint8_t b;
    uint16_t crc_calculated, incoming_crc;

    available_bytes = uart_read->available(); // how many bytes of data are available in the serial buffer?

    if (available_bytes >= READ_BUFFER_SIZE+1) { //check that we have received all bytes from strain data. +1 is to check that we have also received start byte
        for (uint8_t byte = 0; byte < (available_bytes - (READ_BUFFER_SIZE+1)); byte++) { //run through all available bytes
            b = uart_read->read(); //read each byte in the buffer
            if (b == 255){ // check if current byte is equal to start byte (0xFF)
                for (uint8_t data_byte = 0; data_byte < READ_BUFFER_SIZE; data_byte++) {
                    read_buffer[data_byte] = uart_read->read(); 
                }
                incoming_crc = uart_read->read();

                uart_read->flush();// flush all remaining data

                crc_calculated = crc_crc16_ibm(0, bufferp, NUM_SENSORSS * BYTES_PER_SENSOR); 

                if (incoming_crc == crc_calculated) {
                    return true; // we have been able to read an entire packet of data
                } else {
                    //set some flag that is a member of the class
                    return false;
                }
            }
        } 
        return false; // could not find start byte after looping through all available bytes
    } 
    return false; // not enough bytes available. Won't get a full reading
} 

void UARTTest::Log_Strain(uint32_t *strain_array) {
    
    struct log_Strain1 pkt1 = {
        LOG_PACKET_HEADER_INIT(LOG_STRAIN_MSG1),
        time_us :   AP_HAL::micros64(),
        v1      :   strain_array[0],
        v2      :   strain_array[1],
        v3      :   strain_array[2],
        v4      :   strain_array[3],
        v5      :   strain_array[4],
        v6      :   strain_array[5],
        v7      :   strain_array[6],
        v8      :   strain_array[7],
        v9      :   strain_array[8],
        v10     :   strain_array[9],
        v11     :   strain_array[10]
    };

    struct log_Strain2 pkt2 = {
        LOG_PACKET_HEADER_INIT(LOG_STRAIN_MSG2),
        time_us :   AP_HAL::micros64(),
        v12     :    strain_array[11],
        v13     :    strain_array[12],
        v14     :    strain_array[13],
        v15     :    strain_array[14],
        v16     :    strain_array[15],
        v17     :    strain_array[16],
        v18     :    strain_array[17],
        v19     :    strain_array[18],
        v20     :    strain_array[19],
        v21     :    strain_array[20],
        v22     :    strain_array[21],
    };

    struct log_Strain3 pkt3 = {
        LOG_PACKET_HEADER_INIT(LOG_STRAIN_MSG3),
        time_us :   AP_HAL::micros64(),
        v23     :   strain_array[22],
        v24     :   strain_array[23],
        v25     :   strain_array[24],
        v26     :   strain_array[25],
        v27     :   strain_array[26],
        v28     :   strain_array[27],
        v29     :   strain_array[28],
        v30     :   strain_array[29],
        v31     :   strain_array[30],
        v32     :   strain_array[31]
    };

    AP::logger().WriteBlock(&pkt1, sizeof(pkt1));
    AP::logger().WriteBlock(&pkt2, sizeof(pkt2));
    AP::logger().WriteBlock(&pkt3, sizeof(pkt3));
}
