#include "ModbusRTU.h"


#if USE_CRC_TABLE
    #if USE_FLASH
        #define CRC_TABLE_ATTR FLASH_ATTR
        #define CRC_TABLE_READ(idx) ((uint16_t)FLASH_READ_WORD(&crc_table[(idx)]))
    #else
        #define CRC_TABLE_ATTR
        #define CRC_TABLE_READ(idx) (crc_table[(idx)])
    #endif

    static const uint16_t crc_table[256] CRC_TABLE_ATTR = {
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

#else
    /**
     * @brief Computes a single Modbus CRC lookup value without keeping the
     * full 256-entry table in memory.
     *
     * This is useful on very small MCUs where RAM/flash is tight.
     */
    static inline uint16_t calculateCRCEntry(uint8_t index){
        uint16_t crc = (uint16_t)index;
        for (uint8_t bit = 0; bit < 8; ++bit){
            if ((crc & 0x0001U) != 0U){
                crc = (uint16_t)((crc >> 1) ^ 0xA001U);
            }
            else {
                crc >>= 1;
            }
        }
        return crc;
    }

    #define CRC_TABLE_READ(idx) (calculateCRCEntry((idx)))
#endif


/**
 * @brief Calculates CRC for MODBUS message.
 *
 * @param packetData Modbus packet in form of raw data
 * @param length Length of buffer (in bytes, excluding CRC)
 * @param response If false, calculated CRC is compared with crc from packet and result is returned.
 * If true, CRC is calculated and stored at the end of message (return value is always true in this case).
 * @return Whether the CRCs match
 */
bool ModbusRTU::calculateCRC(volatile uint8_t* packetData, uint16_t length, bool response)
{
	uint8_t xor0 = 0;
	uint16_t crc = 0xFFFF;

	for (uint16_t i = 0; i < length; ++i)
	{
		xor0 = (uint8_t)(packetData[i] ^ crc);
		crc >>= 8;
		crc ^= CRC_TABLE_READ(xor0);
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
void ModbusRTU::sendData(volatile uint8_t* packetData, uint16_t length){
    if (serialWriteFunction == NULL) return;
    calculateCRC(packetData, length, true);
    serialWriteFunction((const char*)packetData, (uint16_t)(length + CRC_LEN), serialWriteCtx);
}

/**
 * @brief Receives data through serial line
 *
 * @param data Received raw data buffer
 * @param length Length of expected packet (in bytes, excluding CRC)
 *
 * @return Number of received bytes if data were successfully received and CRC has correct value, 0 otherwise
 */
uint16_t ModbusRTU::recvData(volatile uint8_t* data, uint16_t length){
    if (serialReadFunction == NULL) return 0;

    const int recvNum = (int)serialReadFunction((char*)data, (uint16_t)(length + CRC_LEN), serialReadCtx);

    if (recvNum < 5){
        return 0;
    }

    const uint16_t dataLength = (uint16_t)recvNum - CRC_LEN;
    if (calculateCRC(data, dataLength, false) == true){
        return dataLength;
    }
    return 0;
}

#if !USE_CUSTOM_READ_WRITE_FUNCTIONS
    /**
     * @brief Sets ModbusRTU communication parameters.
     *
     * @param address Device address
     * @param baudRate Communication baud rate
     * @param serialPort Serial port to be used
     * @param initialize Set to false if serial port is being initialized manually,
     * outside of this function
     */
    void ModbusRTU::start(uint8_t address, uint32_t baudRate, HardwareSerial& serialPort, bool initialize){
        this->deviceAddress = address;
        defaultSerialCtx.serial = &serialPort;
        if (initialize == true){
            serialPort.begin(baudRate, SERIAL_8E1);
            serialPort.setTimeout(1);
        }
        if (baudRate == 0){
            baudRate = 9600;
        }
        //Character time in microseconds, rounded up: 1 start bit + 8 data bits + parity + 1 stop bit.
        defaultSerialCtx.byteTransTime = (uint32_t)((11UL * 1000000UL + baudRate - 1UL) / baudRate);
    }
#else
    /**
     * @brief Sets ModbusRTU communication parameters.
     *
     * @param address Device address
     * */
    void ModbusRTU::start(uint8_t address){
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
    //Read each field from the packet once, then keep working with the local copies.
    const uint16_t firstRegister = endianity_swap_16bit(packet->first_register);
    const uint16_t registerCount = endianity_swap_16bit(packet->register_count);
    const uint8_t functionCode = packet->function_code;
    packet->first_register = firstRegister;
    packet->register_count = registerCount;

    uint16_t* registers;
    uint16_t registerNum;
    void(*callback) (uint8_t* buffer, uint16_t bufferLen, void* ctx);
    void* callbackCtx;

    if (functionCode == FC_READ_INPUT_REGISTERS){
        registers = inputRegisters;
        registerNum = INPUT_REGISTER_NUM;
        callback = readInputRegistersCallback;
        callbackCtx = readInputRegistersCallbackCtx;
    }
    else {
        registers = holdingRegisters;
        registerNum = HOLDING_REGISTER_NUM;
        callback = readHoldingRegistersCallback;
        callbackCtx = readHoldingRegistersCallbackCtx;
    }

    //No buffer was ever supplied (with USE_EXTERNAL_REGISTER_BUFFERS).
    if (registers == NULL){
        sendErrorResponse(packet, EX_ILLEGAL_ADDRESS);
        return;
    }

    if (registerCount == 0 || registerCount > MAX_READ_REGISTER_COUNT){
        sendErrorResponse(packet, EX_ILLEGAL_DATA_VALUE);
        return;
    }

    if ((uint32_t)firstRegister + (uint32_t)registerCount > (uint32_t)registerNum){
        sendErrorResponse(packet, EX_ILLEGAL_ADDRESS);
        return;
    }

    const uint16_t dataBytes = (uint16_t)(registerCount * 2);
    const uint16_t frameLength = (uint16_t)(MODBUS_RESPONSE_BASE_LENGTH + dataBytes);
    const uint16_t bufferSize = (uint16_t)(frameLength + CRC_LEN);

    #if USE_FIXED_SCRATCH_BUFFER
        if (bufferSize > SCRATCH_BUFFER_SIZE){
            sendErrorResponse(packet, EX_ILLEGAL_DATA_VALUE);
            return;
        }
        uint8_t* mbResponse = scratchBuffer;
    #else
        uint8_t mbResponse[bufferSize];
    #endif
    memset(mbResponse, 0, bufferSize);

    mbResponse[0] = packet->address;
    mbResponse[1] = functionCode;
    mbResponse[2] = (uint8_t)dataBytes; //Number of bytes to follow

    for (uint16_t i = 0; i < registerCount; ++i){
        put_16bit_into_byte_buffer(mbResponse, (uint16_t)(MODBUS_RESPONSE_BASE_LENGTH + (2 * i)),
            endianity_swap_16bit(registers[firstRegister + i]));
    }

    if (callback != NULL){
        callback(mbResponse, frameLength, callbackCtx);
    }
    sendData(mbResponse, frameLength);
}


/**
 * @brief Handles Write_Single_Register request and sends response.
 *
 * @param packet Modbus packet
 * @return True if request was successfully handled, false otherwise
 */
bool ModbusRTUServer::writeRegisterHandler(volatile requestPacket* packet){
    const uint16_t firstRegister = endianity_swap_16bit(packet->first_register);
    const uint16_t registerValue = endianity_swap_16bit(packet->single_register_data);

    if (firstRegister >= HOLDING_REGISTER_NUM){
        sendErrorResponse(packet, EX_ILLEGAL_ADDRESS);
        return false;
    }

    packet->first_register = firstRegister;
    packet->single_register_data = registerValue;

    if (writeHoldingRegisterCallback != NULL){
        writeHoldingRegisterCallback((uint8_t*)packet, MODBUS_REQUEST_BASE_LENGTH, writeHoldingRegisterCallbackCtx);
    }
    holdingRegisters[firstRegister] = registerValue;

    packet->first_register = endianity_swap_16bit(firstRegister);
    packet->single_register_data = endianity_swap_16bit(registerValue);
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
void ModbusRTUServer::setMultipleInputRegistersValues(uint16_t startAddress, uint16_t* data, uint16_t length){
    if (data == NULL) return;
    if ((uint32_t)startAddress + (uint32_t)length <= (uint32_t)INPUT_REGISTER_NUM){
        for (uint16_t i = 0; i < length; i++){
            inputRegisters[startAddress + i] = data[i];
        }
    }
}

void ModbusRTUServer::setMultipleHoldingRegistersValues(uint16_t startAddress, uint16_t* data, uint16_t length){
    if (data == NULL) return;
    if ((uint32_t)startAddress + (uint32_t)length <= (uint32_t)HOLDING_REGISTER_NUM){
        for (uint16_t i = 0; i < length; i++){
            holdingRegisters[startAddress + i] = data[i];
        }
    }
}

void ModbusRTUServer::getMultipleHoldingRegistersValues(uint16_t startAddress, uint16_t* data, uint16_t length){
    if (data == NULL) return;
    if ((uint32_t)startAddress + (uint32_t)length <= (uint32_t)HOLDING_REGISTER_NUM){
        for (uint16_t i = 0; i < length; i++){
            data[i] = holdingRegisters[startAddress + i];
        }
    }
}




/**
 * @brief Executes read data transaction
 * @param packet Request packet
 * @param registers Received data registers
 * @param timeout Transaction timeout (in microseconds)
 * @param allowException Whether the exception should be treated as a valid transaction
 *
 * @return 0 if response was successfully received, -1 otherwise, positive exception code in case of exception
 */
int ModbusRTUClient::readRegisters(requestPacket* packet, uint16_t* registers, uint32_t timeout, bool allowException){
    if (registers == NULL){
        return -1;
    }

    const uint16_t registerCount = packet->register_count;
    if (registerCount == 0 || registerCount > MAX_READ_REGISTER_COUNT){
        return -1;
    }

    const uint16_t dataBytes = (uint16_t)(registerCount * 2);
    const uint16_t frameLength = (uint16_t)(MODBUS_RESPONSE_BASE_LENGTH + dataBytes);
    const uint16_t bufferSize = (uint16_t)(frameLength + CRC_LEN);

    #if USE_FIXED_SCRATCH_BUFFER
        if (bufferSize > SCRATCH_BUFFER_SIZE){
            return -1;
        }
        uint8_t* mbResponse = scratchBuffer;
    #else
        uint8_t mbResponse[bufferSize];
    #endif
    memset(mbResponse, 0, bufferSize);

    #if !USE_CUSTOM_READ_WRITE_FUNCTIONS
        defaultSerialCtx.setTransTimeout(frameLength);
        defaultSerialCtx.SetRespTimeout(timeout);
    #endif

    packet->first_register = endianity_swap_16bit(packet->first_register);
    packet->register_count = endianity_swap_16bit(packet->register_count);
    sendData(packet->raw_data, MODBUS_REQUEST_BASE_LENGTH);

    const uint16_t readBytes = recvData(mbResponse, frameLength);
    if (readBytes == frameLength){
        //Test if response is correct
        if (mbResponse[0] != packet->address || mbResponse[1] != packet->function_code || mbResponse[2] != dataBytes){
            return -1;
        }
        for (uint16_t i = 0; i < registerCount; i++){
            registers[i] = endianity_swap_16bit(get_16bit_from_byte_buffer(mbResponse, (uint16_t)(MODBUS_RESPONSE_BASE_LENGTH + (i * 2))));
        }
        return 0;
    }
    else if (allowException == true && readBytes == MODBUS_RESPONSE_BASE_LENGTH &&
             mbResponse[0] == packet->address &&
             mbResponse[1] == (uint8_t)(packet->function_code | 0b10000000)){
        return mbResponse[2];
    }
    return -1;
}

/**
 * @brief Executes write transaction
 * @param packet Request packet
 * @param registers Request registers
 * @param timeout Transaction timeout (in microseconds)
 * @param allowException Whether the exception should be treated as a valid transaction
 *
 * @return 0 if response was successfully received, -1 otherwise, positive exception code in case of exception
 */
int ModbusRTUClient::writeRegisters(requestPacket* packet, uint16_t* registers, uint32_t timeout, bool allowException){
    const bool multipleRegisters = (packet->function_code == FC_WRITE_MULTIPLE_REGISTERS);

    const uint16_t registerCount = (registers == NULL) ? 0 : packet->register_count;
    if (multipleRegisters == true &&
        (registers == NULL || registerCount == 0 || registerCount > MAX_WRITE_REGISTER_COUNT)){
        return -1;
    }

    const uint16_t frameLength = (uint16_t)(MODBUS_REQUEST_BASE_LENGTH +
        (multipleRegisters ? (uint16_t)(1 + registerCount * 2) : (uint16_t)0));
    const uint16_t bufferSize = (uint16_t)(frameLength + CRC_LEN);

    #if USE_FIXED_SCRATCH_BUFFER
        if (bufferSize > SCRATCH_BUFFER_SIZE){
            return -1;
        }
        uint8_t* mbRequest = scratchBuffer;
    #else
        uint8_t mbRequest[bufferSize];
    #endif
    memset(mbRequest, 0, bufferSize);

    #if !USE_CUSTOM_READ_WRITE_FUNCTIONS
        defaultSerialCtx.setTransTimeout(MODBUS_REQUEST_BASE_LENGTH);
        defaultSerialCtx.SetRespTimeout(timeout);
    #endif

    mbRequest[0] = packet->address;
    mbRequest[1] = packet->function_code;
    packet->first_register = endianity_swap_16bit(packet->first_register);
    packet->single_register_data = endianity_swap_16bit(packet->single_register_data);
    put_16bit_into_byte_buffer(mbRequest, 2, packet->first_register);
    put_16bit_into_byte_buffer(mbRequest, 4, packet->single_register_data);
    if (multipleRegisters == true){
        mbRequest[6] = (uint8_t)(registerCount * 2);
        for (uint16_t i = 0; i < registerCount; i++){
            put_16bit_into_byte_buffer(mbRequest, (uint16_t)(MODBUS_REQUEST_BASE_LENGTH + 1 + (i * 2)), endianity_swap_16bit(registers[i]));
        }
    }
    sendData(mbRequest, frameLength);

    uint8_t mbResponse[MODBUS_REQUEST_BASE_LENGTH + CRC_LEN] = {0};
    const uint16_t readBytes = recvData(mbResponse, MODBUS_REQUEST_BASE_LENGTH);
    if (readBytes == MODBUS_REQUEST_BASE_LENGTH){
        //Test if response is correct
        if (memcmp(mbResponse, mbRequest, MODBUS_REQUEST_BASE_LENGTH) != 0){
            return -1;
        }
        return 0;
    }
    else if (allowException == true && readBytes == MODBUS_RESPONSE_BASE_LENGTH &&
             mbResponse[0] == packet->address &&
             mbResponse[1] == (uint8_t)(packet->function_code | 0b10000000)){
        return mbResponse[2];
    }
    return -1;
}



//Transaction handlers
int ModbusRTUClient::ReadInputRegisters(uint16_t firstRegister, uint16_t registerNum, uint16_t* registers, uint32_t timeout, bool allowException){
    requestPacket packet = {0};
    packet.address = deviceAddress;
    packet.function_code = FC_READ_INPUT_REGISTERS;
    packet.first_register = firstRegister;
    packet.register_count = registerNum;
    return readRegisters(&packet, registers, timeout * 1000, allowException);
}

int ModbusRTUClient::ReadHoldingRegisters(uint16_t firstRegister, uint16_t registerNum, uint16_t* registers, uint32_t timeout, bool allowException){
    requestPacket packet = {0};
    packet.address = deviceAddress;
    packet.function_code = FC_READ_HOLDING_REGISTERS;
    packet.first_register = firstRegister;
    packet.register_count = registerNum;
    return readRegisters(&packet, registers, timeout * 1000, allowException);
}

int ModbusRTUClient::WriteSingleRegister(uint16_t firstRegister, uint16_t data, uint32_t timeout, bool allowException){
    requestPacket packet = {0};
    packet.address = deviceAddress;
    packet.function_code = FC_WRITE_SINGLE_REGISTER;
    packet.first_register = firstRegister;
    packet.single_register_data = data;
    return writeRegisters(&packet, NULL, timeout * 1000, allowException);
}

int ModbusRTUClient::WriteMultipleRegisters(uint16_t firstRegister, uint16_t registerNum, uint16_t* registers, uint32_t timeout, bool allowException){
    requestPacket packet = {0};
    packet.address = deviceAddress;
    packet.function_code = FC_WRITE_MULTIPLE_REGISTERS;
    packet.first_register = firstRegister;
    packet.register_count = registerNum;
    return writeRegisters(&packet, registers, timeout * 1000, allowException);
}





#if !USE_CUSTOM_READ_WRITE_FUNCTIONS
    /**
     * @brief Default serial read function
     * @param buffer Buffer where data will be stored
     * @param length Number of bytes to receive
     * @param currentCtx Serial port context
     * @return Number of bytes read if data was read successfully, 0 otherwise
     */
    uint16_t defaultSerialReadFunction(char* buffer, uint16_t length, SerialCtx* currentCtx){
        if (currentCtx == NULL || currentCtx->serial == NULL || buffer == NULL){
            return 0;
        }

        uint16_t iterator = 0;
        bool bytesDetected = false;
        uint32_t startTimestamp = GET_TIMESTAMP_US();

        /*Wait until bytes are detected. The line is always polled at least once,
        so a responseTimeout of zero means "look now and return", which is what
        the server needs in order not to block the main program.*/
        do {
            if (currentCtx->serial->available() > 0){
                bytesDetected = true;
                break;
            }

        } while ((uint32_t)(GET_TIMESTAMP_US() - startTimestamp) < currentCtx->responseTimeout);

        //No bytes detected
        if (bytesDetected == false){
            return 0;
        }

        startTimestamp = GET_TIMESTAMP_US();
        while (iterator < length &&
               (uint32_t)(GET_TIMESTAMP_US() - startTimestamp) < currentCtx->transactionTimeout){
            if (currentCtx->serial->available() > 0){
                iterator += (uint16_t)currentCtx->serial->readBytes(buffer + iterator, 1);
            }
        }

        //A short frame is only usable when an exception response is expected
        const bool lengthValid = (iterator == length) ||
            (currentCtx->acceptException == true && iterator == MODBUS_RESPONSE_BASE_LENGTH + CRC_LEN);

        //Discard anything left over
        if (lengthValid == false || currentCtx->serial->available() > 0){
            uint8_t discard[1];
            while (currentCtx->serial->available() > 0){
                currentCtx->serial->readBytes(discard, 1);
            }
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
    void defaultSerialWriteFunction(const char* buffer, uint16_t length, SerialCtx* currentCtx){
        if (currentCtx == NULL || currentCtx->serial == NULL || buffer == NULL){
            return;
        }
        currentCtx->serial->write((const uint8_t*)buffer, length);
    }
#endif
