#include "ModbusRTU.h"


/**
 * @brief Calculates CRC for MODBUS message.
 * 
 * @param packet_data Modbus packet in form of raw data
 * @param length Length of buffer (in bytes, excluding CRC)
 * @param response If false, calculated CRC is compared with request crc and result is returned.
 * If true, CRC is calculated and stored at the end of message (return value is always true in this case).
 * @return Whether the CRCs match
 */
bool ModbusRTU::calculateCRC(volatile uint8_t* packetData, uint16_t length, bool response)
{
	uint8_t xor0 = 0;
	uint16_t crc = 0xFFFF;

	for (uint16_t i = 0; i < length; ++i)
	{
		xor0 = packetData[i] ^ crc;
		crc >>= 8;
		crc ^= crc_table[xor0];
	}

	if (response){
        //Stores at the end of packet
        put_16bit_into_byte_buffer(packetData, length, crc);
        return true;
    }
    else {
        return get_16bit_from_byte_buffer(packetData, length) == crc;
    }
}



//Data exchange
/**
 * @brief Sends data through serial line
 * 
 * @param packetData Modbus packet in form of raw data
 * @param length Length of packet (in bytes, excluding CRC)
 */
void ModbusRTU::sendData(volatile uint8_t* packetData, uint8_t length){
    calculateCRC(packetData, length, true);
    serialWriteFunction((const char*)packetData, length + CRC_LEN, serialWriteCtx);
}

/**
 * @brief Receives data through serial line
 * 
 * @param data Received raw data buffer
 * @param length Length of expected packet (in bytes, excluding CRC)
 * 
 * @return Number of received bytes if data were successfully received and CRC has correct value, 0 otherwise
 */
uint8_t ModbusRTU::recvData(volatile uint8_t* data, uint8_t length){
    uint8_t recvNum = serialReadFunction((char*)data, length + CRC_LEN, serialReadCtx);
    if (recvNum > 0){
        recvNum -= CRC_LEN;
        if (calculateCRC(data, recvNum, false) == true){
            return recvNum;
        }
    }
    return 0;
}

#if !USE_CUSTOM_READ_WRITE_FUNCTIONS
    void ModbusRTU::start(uint16_t address, uint32_t baudRate, HardwareSerial& serialPort, bool initialize){
        this->deviceAddress = address;
        defaultSerialCtx.serial = &Serial;
        if (initialize == true){
            Serial.begin(baudRate, SERIAL_8E1);
            Serial.setTimeout(1);
        }
        //Calculate timeout based on baud rate in microseconds
        defaultSerialCtx.byteTransTime = ((int)ceil(1000000 / baudRate)) * (1 + 8 + 1 + 1); //1 start bit + 8 data bits + parity + 1 stop bit
    }
#else
    void ModbusRTU::start(uint16_t address){
        this->deviceAddress = address;
    }
#endif



//Request handlers
/**
 * @brief Sends error response when exception occured
 * 
 * @param packet Modbus packet
 * @param error_code Code of exception
 */
void ModbusRTUServer::sendErrorResponse(volatile requestPacket* packet, uint8_t error_code){
    uint8_t mbResponse[MODBUS_RESPONSE_BASE_LENGTH + CRC_LEN] = {packet->address, (uint8_t)(packet->function_code | 0b10000000), error_code};
    sendData(mbResponse, MODBUS_RESPONSE_BASE_LENGTH);
}

/**
 * @brief Handles Read Registers request and sends response
 * @param packet Modbus packet
 */
void ModbusRTUServer::readRegistersHandler(volatile requestPacket* packet){
    packet->first_register = endianity_swap_16bit(packet->first_register);
    packet->register_count = endianity_swap_16bit(packet->register_count);

    uint16_t* registers;
    uint8_t registerNum;
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

    int bufferSize = MODBUS_RESPONSE_BASE_LENGTH + (packet->register_count * 2) + CRC_LEN;

    if (packet->first_register + packet->register_count > registerNum || 
        packet->register_count > MAX_READ_REGISTER_COUNT){
        sendErrorResponse(packet, EX_ILLEGAL_ADDRESS);
    }

    else if (USE_FIXED_SCRATCH_BUFFER_SIZE && bufferSize > SCRATCH_BUFFER_SIZE){
        sendErrorResponse(packet, EX_ILLEGAL_ADDRESS);
    }

    else {
        #if USE_FIXED_SCRATCH_BUFFER_SIZE
            uint8_t mbResponse[SCRATCH_BUFFER_SIZE] = {0};
        #else
            uint8_t mbResponse[bufferSize] = {0};
        #endif

        mbResponse[0] = packet->address;
        mbResponse[1] = packet->function_code;
        mbResponse[2] = packet->register_count * 2; //Number of bytes to follow

        for (uint16_t i = 0; i < packet->register_count; ++i){
            put_16bit_into_byte_buffer(mbResponse, MODBUS_RESPONSE_BASE_LENGTH + (2 * i), endianity_swap_16bit(registers[packet->first_register + i]));
        }

        if (event != NULL){
            event(mbResponse, MODBUS_RESPONSE_BASE_LENGTH + (packet->register_count * 2), eventCtx);
        }
        sendData(mbResponse, MODBUS_RESPONSE_BASE_LENGTH + (packet->register_count * 2));
    }
}
    

/**
 * @brief Handles Write_Single_Register request and sends response.
 * 
 * @param packet Modbus packet
 * @return True if request was successfully handled, false otherwise
 */
bool ModbusRTUServer::writeRegisterHandler(volatile requestPacket* packet){
    packet->first_register = endianity_swap_16bit(packet->first_register);
    packet->single_register_data = endianity_swap_16bit(packet->single_register_data);

    if (packet->first_register + 1 > HOLDING_REGISTER_NUM){
        sendErrorResponse(packet, EX_ILLEGAL_ADDRESS);
        return false;
    }

    if (writeHoldingRegisterEvent != NULL){
        writeHoldingRegisterEvent((uint8_t*)packet, MODBUS_REQUEST_BASE_LENGTH, writeHoldingRegisterEventCtx);
    }
    holdingRegisters[packet->first_register] = packet->single_register_data;

    packet->first_register = endianity_swap_16bit(packet->first_register);
    packet->single_register_data = endianity_swap_16bit(packet->single_register_data);
    sendData(packet->raw_data, MODBUS_REQUEST_BASE_LENGTH);

    return true;
}

/**
 * @brief Handles incoming Modbus request
 * 
 * @param packet Modbus packet
 * @return True in case of write request (new data), false otherwise
 */
bool ModbusRTUServer::handleRequest(requestPacket* packet){
    bool returnValue = false;
    
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


bool ModbusRTUServer::communicationLoop(){
    
    requestPacket receivedPacket = {0};
    if (recvData(receivedPacket.raw_data, MODBUS_REQUEST_BASE_LENGTH) != MODBUS_REQUEST_BASE_LENGTH || receivedPacket.address != deviceAddress){
        return false;
    }
    return handleRequest(&receivedPacket);
}



//Data handlers
void ModbusRTUServer::copyToInputRegisters(uint16_t* data, uint16_t length, uint16_t startAddress){
    if (startAddress + length <= INPUT_REGISTER_NUM){
        for (uint16_t i = 0; i < length; i++){
            inputRegisters[startAddress + i] = data[i];
        }
    }
}

void ModbusRTUServer::copyToHoldingRegisters(uint16_t* data, uint16_t length, uint16_t startAddress){
    if (startAddress + length <= HOLDING_REGISTER_NUM){
        for (uint16_t i = 0; i < length; i++){
            holdingRegisters[startAddress + i] = data[i];
        }
    }
}

void ModbusRTUServer::copyFromHoldingRegisters(uint16_t* data, uint16_t length, uint16_t startAddress){
    if (startAddress + length <= HOLDING_REGISTER_NUM){
        for (uint16_t i = 0; i < length; i++){
            data[i] = holdingRegisters[startAddress + i];
        }
    }
}





/**
 * @brief Executes read transaction
 * @param packet Request packet
 * @param registers Received data registers
 * @param timeout Transaction timeout (in microseconds)
 * @param allowExceptions Whether treat exception as valid transaction
 * 
 * @return 0 if response were successfully received, -1 otherwise, positive exception code in case of exception
 */
int ModbusRTUClient::readRegisters(requestPacket* packet, uint16_t* registers, uint32_t timeout, bool allowException){
    uint8_t registerCount = packet->register_count;
    if (registerCount > MAX_READ_REGISTER_COUNT){
        return -1;
    }

    #if !USE_CUSTOM_READ_WRITE_FUNCTIONS
        defaultSerialCtx.setTransTimeout(registerCount * 2 + MODBUS_RESPONSE_BASE_LENGTH);
        defaultSerialCtx.SetRespTimeout(timeout);
    #endif    
    
    int bufferSize = MODBUS_RESPONSE_BASE_LENGTH + registerCount * 2 + CRC_LEN;
    #if USE_FIXED_SCRATCH_BUFFER_SIZE
        uint8_t mbResponse[SCRATCH_BUFFER_SIZE] = {0};
    #else
        uint8_t mbResponse[bufferSize] = {0};
    #endif
    if (USE_FIXED_SCRATCH_BUFFER_SIZE && bufferSize > SCRATCH_BUFFER_SIZE){
        return -1;
    }

    packet->first_register = endianity_swap_16bit(packet->first_register);
    packet->register_count = endianity_swap_16bit(packet->register_count);
    sendData(packet->raw_data, MODBUS_REQUEST_BASE_LENGTH);


    uint8_t readBytes = recvData(mbResponse, registerCount * 2 + MODBUS_RESPONSE_BASE_LENGTH);
    if (readBytes == registerCount * 2 + MODBUS_RESPONSE_BASE_LENGTH){
        //Test if response is correct
        if (mbResponse[0] != packet->address || mbResponse[1] != packet->function_code || mbResponse[2] != registerCount * 2){
            return -1;
        }
        for (uint8_t i = 0; i < registerCount; i++){
            registers[i] = endianity_swap_16bit(get_16bit_from_byte_buffer(mbResponse, MODBUS_RESPONSE_BASE_LENGTH + (i * 2)));
        }
        return 0;
    }
    else if (allowException == true && readBytes == MODBUS_RESPONSE_BASE_LENGTH && mbResponse[1] == (packet->function_code | 0b10000000)){
        return mbResponse[2];
    }
    return -1;
}

/**
 * @brief Executes write transaction
 * @param packet Request packet
 * @param registers Request registers
 * @param timeout Transaction timeout (in microseconds)
 * @param allowExceptions Whether treat exception as valid transaction
 * 
 * @return 0 if response were successfully received, -1 otherwise, positive exception code in case of exception
 */
int ModbusRTUClient::writeRegisters(requestPacket* packet, uint16_t* registers, uint32_t timeout, bool allowException){
    uint8_t registerCount = registers == NULL ? 0 : packet->register_count;
    if (registerCount > MAX_WRITE_REGISTER_COUNT){
        return -1;
    }
    bool multipleRegisters = packet->function_code == FC_WRITE_MULTIPLE_REGISTERS;

    #if !USE_CUSTOM_READ_WRITE_FUNCTIONS
        defaultSerialCtx.setTransTimeout(MODBUS_REQUEST_BASE_LENGTH);
        defaultSerialCtx.SetRespTimeout(timeout);
    #endif

    int bufferSize = MODBUS_REQUEST_BASE_LENGTH + multipleRegisters + registerCount * 2 + CRC_LEN;
    #if USE_FIXED_SCRATCH_BUFFER_SIZE
        uint8_t mbRequest[SCRATCH_BUFFER_SIZE] = {0};
    #else
        uint8_t mbRequest[bufferSize] = {0};
    #endif
    if (USE_FIXED_SCRATCH_BUFFER_SIZE && bufferSize > SCRATCH_BUFFER_SIZE){
        return -1;
    }

    mbRequest[0] = packet->address;
    mbRequest[1] = packet->function_code;
    packet->first_register = endianity_swap_16bit(packet->first_register);
    packet->single_register_data = endianity_swap_16bit(packet->single_register_data);
    put_16bit_into_byte_buffer(mbRequest, 2, packet->first_register);
    put_16bit_into_byte_buffer(mbRequest, 4, packet->single_register_data);
    if (multipleRegisters == true){
        mbRequest[6] = registerCount * 2;
        for (uint16_t i = 0; i < registerCount; i++){
            put_16bit_into_byte_buffer(mbRequest, MODBUS_REQUEST_BASE_LENGTH + 1 + (i * 2), endianity_swap_16bit(registers[i]));
        }
    }
    sendData(mbRequest, bufferSize - CRC_LEN);

    uint8_t mbResponse[MODBUS_REQUEST_BASE_LENGTH + CRC_LEN] = {0};
    int readBytes  = recvData(mbResponse, MODBUS_REQUEST_BASE_LENGTH);
    if (readBytes == MODBUS_REQUEST_BASE_LENGTH){
        //Test if response is correct
        if (memcmp(mbResponse, mbRequest, MODBUS_REQUEST_BASE_LENGTH) != 0){
            return -1;
        }
        return 0;
    }
    else if (allowException == true && readBytes == MODBUS_RESPONSE_BASE_LENGTH && mbResponse[1] == (packet->function_code | 0b10000000)){
        return mbResponse[2];
    }
    return -1;
}


//Transaction handlers
int ModbusRTUClient::ReadInputRegisters(uint16_t firstRegister, uint16_t registerNum, uint16_t* registers, uint32_t timeout, bool allowException){
    requestPacket packet;
    packet.address = deviceAddress;
    packet.function_code = FC_READ_INPUT_REGISTERS;
    packet.first_register = firstRegister;
    packet.register_count = registerNum;
    return readRegisters(&packet, registers, timeout, allowException);
}

int ModbusRTUClient::ReadHoldingRegisters(uint16_t firstRegister, uint16_t registerNum, uint16_t* registers, uint32_t timeout, bool allowException){
    requestPacket packet;
    packet.address = deviceAddress;
    packet.function_code = FC_READ_HOLDING_REGISTERS;
    packet.first_register = firstRegister;
    packet.register_count = registerNum;
    return readRegisters(&packet, registers, timeout, allowException);
}

int ModbusRTUClient::WriteSingleRegister(uint16_t firstRegister, uint16_t data, uint32_t timeout, bool allowException){
    requestPacket packet;
    packet.address = deviceAddress;
    packet.function_code = FC_WRITE_SINGLE_REGISTER;
    packet.first_register = firstRegister;
    packet.register_count = data;
    return writeRegisters(&packet, NULL, timeout, allowException);
}

int ModbusRTUClient::WriteMultipleRegisters(uint16_t firstRegister, uint16_t registerNum, uint16_t* registers, uint32_t timeout, bool allowException){
    requestPacket packet;
    packet.address = deviceAddress;
    packet.function_code = FC_WRITE_MULTIPLE_REGISTERS;
    packet.first_register = firstRegister;
    packet.register_count = registerNum;
    return writeRegisters(&packet, registers, timeout, allowException);
}





#if !USE_CUSTOM_READ_WRITE_FUNCTIONS
    /**
     * @brief Default serial read function
     * @param buffer Buffer where data will be stored
     * @param length Number of bytes to receive
     * @param ctx Serial port context
     * @return Number of bytes read if data was read successfully, 0 otherwise
     */
    uint8_t defaultSerialReadFunction(char* buffer, uint8_t length, SerialCtx* currentCtx){
        //In case of delayed bytes
        uint8_t iterator = 0;
        uint32_t lastTimestamp = 0;
        uint64_t currentTimestamp = 0;
        bool bytesDetected = false;
        
        //Wait until bytes are detected
        lastTimestamp = GET_TIMESTAMP_US();
        do {
            currentTimestamp = GET_TIMESTAMP_US();
            if (currentTimestamp < lastTimestamp){
                //Overflow occurred
                currentTimestamp += UINT32_MAX;
            }
        } while (currentTimestamp - lastTimestamp < currentCtx->responseTimeout && 
            (bytesDetected = !!currentCtx->serial->available()) == false);

        //No bytes detected
        if (bytesDetected == false){
            return 0;
        }

        //Reading the message in specified timeout
        lastTimestamp = GET_TIMESTAMP_US();
        do {
            currentTimestamp = GET_TIMESTAMP_US();
            if (currentTimestamp < lastTimestamp){
                //Overflow occurred
                currentTimestamp += UINT32_MAX;
            }
            
            if (currentCtx->serial->available() >= 1){
                iterator += currentCtx->serial->readBytes(buffer + iterator, 1);
            }
        } while (currentTimestamp - lastTimestamp < currentCtx->transactionTimeout && iterator < length);

        //Incorrect length
        if (iterator != length && iterator != MODBUS_RESPONSE_BASE_LENGTH + CRC_LEN){
            return 0;
        }

        //Flush trailing bytes
        else if (currentCtx->serial->available() > 0){
            uint8_t buf[1];
            while (currentCtx->serial->available() > 0) currentCtx->serial->readBytes(buf, 1); //To flush buffer
            return 0;
        }

        return iterator;
    }

    /**
     * @brief Default serial write function
     * @param buffer Buffer which holds data to be sent
     * @param length Length of data to be sent
     * @param currentCtx Serial port context
     */
    void defaultSerialWriteFunction(const char* buffer, uint8_t length, SerialCtx* currentCtx){
        currentCtx->serial->write((const uint8_t*)buffer, length);
    }
#endif