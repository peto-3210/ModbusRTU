#include "ModbusRTU.h"

//NOTE: Return statement in void function will brick the device

//Registers
uint16_t command_register = 0;
uint16_t inputRegisters[2] = {0};


//Packet struct
typedef union {
    uint8_t raw_data[MODBUS_REQUEST_BASE_LENGTH + CRC_LEN + 1];
    struct {
        uint8_t address;
        uint8_t function_code;
        uint16_t first_register;
        union {
            uint16_t register_count;
            uint16_t single_register_data;
        };
        //Last register will hold CRC data
        uint16_t crc;
    };
} request_packet; 


//CRC table
static const uint16_t crc_table[256] = {
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040 };

/**
 * @brief Calculates CRC for MODBUS message.
 * 
 * @param packet_data Modbus packet in form of raw data
 * @param length Length of buffer (in bytes, excluding CRC)
 * @param response If false, calculated CRC is compared with request crc and result is returned.
 * If true, CRC is calculated and stored at the end of message (return value is true).
 * @return Whether the CRCs match
 */
bool calculate_crc(volatile uint8_t* packet_data, uint16_t length, bool response)
{
	uint8_t xor0 = 0;
	uint16_t crc = 0xFFFF;

	for (uint16_t i = 0; i < length; ++i)
	{
		xor0 = packet_data[i] ^ crc;
		crc >>= 8;
		crc ^= crc_table[xor0];
	}

	if (response){
        //Stores at the end of packet
        put_16bit_into_byte_buffer(packet_data, length, crc);
        return true;
    }
    else {
        return get_16bit_from_byte_buffer(packet_data, length) == crc;
    }
}






//Response senders
/**
 * @brief Sends response to received packet
 * 
 * @param packet_data Modbus packet in form of raw data
 * @param length Length of packet (in bytes, excluding CRC)
 */
void send_response(volatile uint8_t* packet_data, uint16_t length){
    calculate_crc(packet_data, length, true);
    Serial.write((const uint8_t*)packet_data, length + CRC_LEN);
}

/**
 * @brief Sends error response when exception occured
 * 
 * @param packet Modbus packet
 * @param error_code Code of exception
 */
void send_error_response(volatile request_packet* packet, uint8_t error_code){
    uint8_t mb_response[MODBUS_READ_RESPONSE_BASE_LEN + CRC_LEN] = {packet->address, (uint8_t)(packet->function_code | 0b10000000), error_code};
    send_response(mb_response, MODBUS_READ_RESPONSE_BASE_LEN);
}





//Request handlers
/**
 * @brief Handles Read_Holding_Registers request and sends response
 */
void readHoldingRegistersHandler(volatile request_packet* packet){
    if (packet->first_register != 0 || packet->register_count != 1){
        send_error_response(packet, EX_ILLEGAL_ADDRESS);
    }

    else {
        uint8_t mb_response[MODBUS_READ_RESPONSE_BASE_LEN + 2 + CRC_LEN] = {0};
        mb_response[0] = packet->address;
        mb_response[1] = packet->function_code;
        mb_response[2] = 2; //Number of bytes to follow
        put_16bit_into_byte_buffer(mb_response, MODBUS_READ_RESPONSE_BASE_LEN, endianity_swap_16bit(command_register));

        send_response(mb_response, MODBUS_READ_RESPONSE_BASE_LEN + 2);
    }
}

/**
 * @brief Handles Read_Input_Registers request and sends response
 */
void readInputRegistersHandler(volatile request_packet* packet){
    //Read input data
    if (packet->register_count + packet->first_register <= 2){
        uint8_t mb_response[MODBUS_READ_RESPONSE_BASE_LEN + 4 + CRC_LEN] = {0};
        mb_response[0] = packet->address;
        mb_response[1] = packet->function_code;
        mb_response[2] = packet->register_count * 2 ; //Number of bytes to follow

        for (uint16_t i = 0; i < packet->register_count; ++i){
            put_16bit_into_byte_buffer(mb_response, MODBUS_READ_RESPONSE_BASE_LEN + 2*i, endianity_swap_16bit(inputRegisters[packet->first_register + i]));
        }

        send_response(mb_response, MODBUS_READ_RESPONSE_BASE_LEN + packet->register_count * 2);
    }

    else {
        send_error_response(packet, EX_ILLEGAL_ADDRESS);
    }
}
    

/**
 * @brief Handles Write_Single_Register request, waits until commands are parsed and sends response.
 */
int16_t writeSingleRegisterHandler(volatile request_packet* packet){
    if (packet->first_register != 0){
        send_error_response(packet, EX_ILLEGAL_ADDRESS);
        return -1;
    }

    command_register = packet->single_register_data;

    packet->first_register = endianity_swap_16bit(packet->first_register);
    packet->register_count = endianity_swap_16bit(packet->register_count);
    send_response(packet->raw_data, MODBUS_REQUEST_BASE_LENGTH);

    return command_register;
}

int16_t handleRequest(request_packet* packet){
    packet->first_register = endianity_swap_16bit(packet->first_register);
    packet->register_count = endianity_swap_16bit(packet->register_count);
    int16_t returnValue = -1;
    
    switch (packet->function_code){
        case FC_READ_HOLDING_REGISTERS:
            readHoldingRegistersHandler(packet);
            break;
        case FC_READ_INPUT_REGISTERS:
            readInputRegistersHandler(packet);
            break;
        case FC_WRITE_SINGLE_REGISTER:
            returnValue = writeSingleRegisterHandler(packet);
            break;
        default:
            send_error_response(packet, EX_ILLEGAL_FUNCTION);
    }
    return returnValue;
}



//Initializers
void initModbus(){
    Serial.begin(BAUD_RATE, CONFIG);
}



int16_t communicationLoop(){
    if (Serial.available() != MODBUS_REQUEST_BASE_LENGTH + CRC_LEN){
        Serial.flush();
        return -1;
    }

    request_packet received_packet = {0};
    Serial.readBytes((char*)&received_packet.raw_data, MODBUS_REQUEST_BASE_LENGTH + CRC_LEN);

    if (received_packet.address == MY_ADDRESS &&
        calculate_crc(received_packet.raw_data, MODBUS_REQUEST_BASE_LENGTH, false) == true){

        return handleRequest(&received_packet);
    }
    return -1;
}
