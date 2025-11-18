#include "ModbusRTU.h"


/**
 * @brief Calculates CRC for MODBUS message.
 * 
 * @param packet_data Modbus packet in form of raw data
 * @param length Length of buffer (in bytes, excluding CRC)
 * @param response If false, calculated CRC is compared with request crc and result is returned.
 * If true, CRC is calculated and stored at the end of message (return value is true).
 * @return Whether the CRCs match
 */
bool ModbusRTU::calculateCRC(volatile uint8_t* packet_data, uint16_t length, bool response)
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
void ModbusRTU::sendResponse(volatile uint8_t* packet_data, uint16_t length){
    calculateCRC(packet_data, length, true);
    serialWriteFunction((const char*)packet_data, length + CRC_LEN, serialWriteCtx);
}

/**
 * @brief Sends error response when exception occured
 * 
 * @param packet Modbus packet
 * @param error_code Code of exception
 */
void ModbusRTU::sendErrorResponse(volatile request_packet* packet, uint8_t error_code){
    uint8_t mb_response[MODBUS_RESPONSE_BASE_LEN + CRC_LEN] = {packet->address, (uint8_t)(packet->function_code | 0b10000000), error_code};
    sendResponse(mb_response, MODBUS_RESPONSE_BASE_LEN);
}



//Request handlers
/**
 * @brief Handles Read Registers request and sends response
 * @param packet Modbus packet
 */
void ModbusRTU::readRegistersHandler(volatile request_packet* packet){
    packet->first_register = endianity_swap_16bit(packet->first_register);
    packet->register_count = endianity_swap_16bit(packet->register_count);

    uint16_t* registers;
    uint16_t registerNum;
    void(*event) (uint8_t* buffer, uint16_t bufferLen, void* ctx);
    void* eventCtx;

    if (packet->function_code == FC_READ_INPUT_REGISTERS){
        registers = inputRegisters;
        registerNum = INPUT_REGISTER_NUM;
        event = readInputRegistersEvent;
        eventCtx = readInputRegistersEventCtx;
    }
    else {
        registers = holdingRegisters;
        registerNum = HOLDING_REGISTER_NUM;
        event = readHoldingRegistersEvent;
        eventCtx = readHoldingRegistersEventCtx;
    }


    if (packet->first_register + packet->register_count > registerNum || 
        packet->register_count > MAX_READ_REGISTER_COUNT){
        sendErrorResponse(packet, EX_ILLEGAL_ADDRESS);
    }

    uint8_t mb_response[MODBUS_RESPONSE_BASE_LEN + (packet->register_count * 2) + CRC_LEN] = {0};
    mb_response[0] = packet->address;
    mb_response[1] = packet->function_code;
    mb_response[2] = packet->register_count * 2; //Number of bytes to follow
    //Serial.print(inputRegisters[0], 10);
    //Serial.flush();
    //delay(10);

    for (uint16_t i = 0; i < packet->register_count; ++i){
        put_16bit_into_byte_buffer(mb_response, MODBUS_RESPONSE_BASE_LEN + (2 * i), endianity_swap_16bit(registers[packet->first_register + i]));
    }


    if (event != NULL){
        event(mb_response, MODBUS_RESPONSE_BASE_LEN + (packet->register_count * 2), eventCtx);
    }

    sendResponse(mb_response, MODBUS_RESPONSE_BASE_LEN + (packet->register_count * 2));
}
    

/**
 * @brief Handles Write_Single_Register request and sends response.
 * 
 * @param packet Modbus packet
 * @return int16_t Written value, -1 if error occured
 */
int16_t ModbusRTU::writeRegisterHandler(volatile request_packet* packet){
    packet->first_register = endianity_swap_16bit(packet->first_register);
    packet->single_register_data = endianity_swap_16bit(packet->single_register_data);

    if (packet->first_register + 1 > HOLDING_REGISTER_NUM){
        sendErrorResponse(packet, EX_ILLEGAL_ADDRESS);
        return -1;
    }

    if (writeHoldingRegisterEvent != NULL){
        writeHoldingRegisterEvent((uint8_t*)packet, MODBUS_REQUEST_BASE_LENGTH, writeHoldingRegisterEventCtx);
    }
    int16_t value = holdingRegisters[packet->first_register] = packet->single_register_data;

    packet->first_register = endianity_swap_16bit(packet->first_register);
    packet->single_register_data = endianity_swap_16bit(packet->single_register_data);
    sendResponse(packet->raw_data, MODBUS_REQUEST_BASE_LENGTH);

    return value;
}

/**
 * @brief Handles incoming Modbus request
 * 
 * @param packet Modbus packet
 * @return int16_t Return value depends on request type:
 * For Read Registers -1
 * For Write Single Register - written value
 */
int16_t ModbusRTU::handleRequest(request_packet* packet){
    int16_t returnValue = -1;
    
    switch (packet->function_code){
        case FC_READ_HOLDING_REGISTERS:
        case FC_READ_INPUT_REGISTERS:
            readRegistersHandler(packet);
            break;
        case FC_WRITE_SINGLE_REGISTER:
            returnValue = writeRegisterHandler(packet);
            break;
        default:
            sendErrorResponse(packet, EX_ILLEGAL_FUNCTION);
    }
    return returnValue;
}

void ModbusRTU::startModbusServer(uint16_t address, unsigned long baudRate){
        this->deviceAddress = address;
        if (defaultSerialCtx.serial == NULL){
            defaultSerialCtx.serial = &Serial;
            Serial.begin(baudRate, SERIAL_8E1);
        }
        //Calculate timeout based on baud rate in microseconds
        defaultSerialCtx.timeout = ((int)ceil(1000000 / baudRate)) * MODBUS_REQUEST_BASE_LENGTH * (1 + 8 + 1 + 1); //1 start bit + 8 data bits + parity + 1 stop bit
        defaultSerialCtx.lastTimestamp = micros();
    }

/**
 * @brief Main communication loop. Call this function periodically.
 * 
 * @return int16_t New data, -1 if none has arrived
 */
int16_t ModbusRTU::communicationLoop(){
    
    request_packet received_packet = {0};
    if (serialReadFunction((char*)received_packet.raw_data, serialReadCtx) == false){
        return -1;
    }

    if (received_packet.address == deviceAddress &&
        calculateCRC(received_packet.raw_data, MODBUS_REQUEST_BASE_LENGTH, false) == true){

        return handleRequest(&received_packet);
    }
    return -1;
}

void ModbusRTU::copyToInputRegisters(uint16_t* data, uint16_t length, uint16_t startAddress){
        if (startAddress + length <= INPUT_REGISTER_NUM){
            for (uint16_t i = 0; i < length; i++){
                inputRegisters[startAddress + i] = data[i];
            }
        }
    }

void ModbusRTU::copyToHoldingRegisters(uint16_t* data, uint16_t length, uint16_t startAddress){
        if (startAddress + length <= HOLDING_REGISTER_NUM){
            for (uint16_t i = 0; i < length; i++){
                holdingRegisters[startAddress + i] = data[i];
            }
        }
    }

void ModbusRTU::copyFromHoldingRegisters(uint16_t* data, uint16_t length, uint16_t startAddress){
        if (startAddress + length <= HOLDING_REGISTER_NUM){
            for (uint16_t i = 0; i < length; i++){
                data[i] = holdingRegisters[startAddress + i];
            }
        }
    }

/**
 * @brief Default serial read function
 * @param buffer Buffer where data will be stored
 * @param ctx Serial port context
 * @return true If data was read successfully, false otherwise
 */
bool defaultSerialReadFunction(char* buffer, void* ctx){
    SerialCtx* currentCtx = (SerialCtx*)ctx;
    HardwareSerial* serialPort = (HardwareSerial*)currentCtx->serial;

    if (serialPort->available() == MODBUS_REQUEST_BASE_LENGTH + CRC_LEN){
        serialPort->readBytes(buffer, MODBUS_REQUEST_BASE_LENGTH + CRC_LEN);
        //digitalWrite(7, toggle2);
        currentCtx->lastTimestamp = micros();
        currentCtx->newDataDetected = false;
        return true;
    }

    if (serialPort->available() == 0){
        currentCtx->lastTimestamp = micros();
        return false;
    }

    unsigned long long currentTimestamp = micros();
    if (currentTimestamp < currentCtx->lastTimestamp){
        //Overflow occurred
        currentTimestamp += UINT32_MAX;
    }

    //If buffer is not empty and timeout has passed, clear buffer
    if (currentCtx->newDataDetected == true && 
        currentTimestamp - currentCtx->lastTimestamp >= currentCtx->timeout){
        while(serialPort->read() != -1); //Clear buffer
        currentCtx->newDataDetected = false;
    }
    else {
        currentCtx->newDataDetected = true;
    }

    currentCtx->lastTimestamp = micros();
    return false;
}

/**
 * @brief Default serial write function
 * @param buffer Buffer which holds data to be sent
 * @param length Length of data to be sent
 * @param ctx Serial port context
 */
void defaultSerialWriteFunction(const char* buffer, uint16_t length, void* ctx){
    HardwareSerial* serialPort = (HardwareSerial*)((SerialCtx*)ctx)->serial;
    serialPort->write((const uint8_t*)buffer, length);
}
