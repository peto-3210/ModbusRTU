#include <Arduino.h>
#include <ctype.h>
#include <string.h>

/*Modbus is implemented as non-inverted UART with even parity and 1 stop bit (according to standard). 
Only ReadInputRegisters, ReadHoldingRegisters and WriteSingleRegister functions are implemented, so the
standard request packet should consist of 6 bytes + CRC (2 bytes). This can be utilized in various cases,
like DMA reading, etc. Protocol data, such as register content, are transmitted in big endian.
*/

//Adjust if necessary
#define INPUT_REGISTER_NUM 100
#define HOLDING_REGISTER_NUM 100
#define USE_EXTERNAL_INPUT_REGISTER_BUFFER false
#define USE_EXTERNAL_HOLDING_REGISTER_BUFFER false
#define USE_CUSTOM_READ_WRITE_FUNCTIONS false
#define USE_FIXED_SCRATCH_BUFFER_SIZE false
#define SCRATCH_BUFFER_SIZE 256
#define GET_TIMESTAMP_US micros



//ModbusRTU defines (do not change)
#define MODBUS_REQUEST_BASE_LENGTH 6
#define MODBUS_RESPONSE_BASE_LENGTH 3
#define CRC_LEN 2


#if USE_FIXED_SCRATCH_BUFFER_SIZE && SCRATCH_BUFFER_SIZE < 8
    #error "SCRATCH_BUFFER_SIZE must be at least 8 bytes!"
#endif

#define FC_READ_HOLDING_REGISTERS 3
#define FC_READ_INPUT_REGISTERS 4
#define FC_WRITE_SINGLE_REGISTER 6
#define FC_WRITE_MULTIPLE_REGISTERS 16

#define EX_ILLEGAL_FUNCTION 1
#define EX_ILLEGAL_ADDRESS 2
//#define EX_SERVER_BUSY 6
#define MAX_READ_REGISTER_COUNT 125
#define MAX_WRITE_REGISTER_COUNT 123

//Used to put 16-bit value into buffer of bytes
#define put_16bit_into_byte_buffer(buffer, offset, value) {(buffer)[(offset) + 1] = ((value) & 0xff00) >> 8; (buffer)[(offset)] = (value) & 0xff;}

//Used to get 16-bit value from buffer of bytes
#define get_16bit_from_byte_buffer(buffer, offset) (((uint16_t)((buffer)[(offset) + 1]) << 8) | (buffer)[(offset)])

//Used to swap endianity
#define endianity_swap_16bit(value) ((uint16_t)(((value) & 0xff) << 8) | (((value) & 0xff00) >> 8))

/**
 * @brief Structure for received modbus packet
 */
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
} requestPacket; 


#if !USE_CUSTOM_READ_WRITE_FUNCTIONS
    /**
     * @brief Default context structure for read and write function
     */
    typedef struct {
        HardwareSerial* serial;
        uint32_t byteTransTime;
        uint32_t transactionTimeout;
        uint32_t responseTimeout;
        bool acceptException; //If true, 5-bytes long transaction will be considered valid, for it may be Modbus exception response

        /**
         * @brief Sets transaction timeout according to number of expected bytes (excluding CRC)
         */
        void setTransTimeout(uint16_t byteNum){
            transactionTimeout = byteTransTime * (byteNum + CRC_LEN + 5); //Add some extra time, just in case
        }

        /**
         * @brief Sets timeout for response
         */
        void SetRespTimeout(uint32_t timeout){
            responseTimeout = timeout;
        }
    } SerialCtx;

    extern uint8_t defaultSerialReadFunction(char* buffer, uint8_t length, SerialCtx* ctx);
    extern void defaultSerialWriteFunction(const char* buffer, uint8_t length, SerialCtx* ctx);
#endif


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
 * @brief Default class
 */
class ModbusRTU{
    protected:
    uint8_t deviceAddress;

    #if !USE_CUSTOM_READ_WRITE_FUNCTIONS
    SerialCtx defaultSerialCtx{NULL, 0, 0, false};
    SerialCtx* serialReadCtx = &defaultSerialCtx;
    SerialCtx* serialWriteCtx = &defaultSerialCtx;
    uint8_t (*serialReadFunction)(char* buffer, uint8_t length, SerialCtx* ctx) = defaultSerialReadFunction;
    void (*serialWriteFunction)(const char* buffer, uint8_t length, SerialCtx* ctx) = defaultSerialWriteFunction;

    #else
    void* serialReadCtx = NULL;
    void* serialWriteCtx = NULL;
    int (*serialReadFunction)(char* buffer, uint16_t length, void* ctx) = NULL;
    void (*serialWriteFunction)(const char* buffer, uint16_t length, void* ctx) = NULL;
    #endif

    protected:
    ModbusRTU(){}
    
    #if USE_CUSTOM_READ_WRITE_FUNCTIONS
    /**
     * @brief Sets custom serial read function
     * This function must accept two parameters: pointer to buffer, where data will be stored
     * and pointer to context (serial port object, or any other user-defined data)
     * Function must return number of read bytes if data was read successfully, -1 otherwise
     * @param readFunction Function pointer to custom read function
     * @param readCtx User-defined context, which will be passed to read function
     */
    void setSerialReadFunction(int (*readFunction)(char* buffer, uint16_t length, void* ctx), void* readCtx){
        serialReadFunction = readFunction;
        serialReadCtx = readCtx;
    }

    /**
     * @brief Sets custom serial write function
     * This function must accept three parameters: pointer to buffer, which holds data to be sent,
     * length of data to be sent and pointer to context (serial port object, or any other user-defined data)
     * @param writeFunction Function pointer to custom write function
     * @param writeCtx User-defined context, which will be passed to write function
     */
    void setSerialWriteFunction(void (*writeFunction)(const char* buffer, uint16_t length,  void* ctx), void* writeCtx){
        serialWriteFunction = writeFunction;
        serialWriteCtx = writeCtx;
    }

    /**
     * @brief Sets ModbusRTU communication parameters. 
     * 
     * @param address Device Modbus address
     * */
    void start(uint16_t address);

    #else

    /**
     * @brief Sets ModbusRTU communication parameters. 
     * 
     * @param address Device Modbus address
     * @param baudRate Communication baud rate
     * @param serialPort HardwareSerial port to be used (default: Serial)
     * @param initialize If true, serial port will be initialized in this function (default: true)
     */
    void start(uint16_t address, uint32_t baudRate, HardwareSerial& serialPort, bool initialize);
    #endif

    bool calculateCRC(volatile uint8_t* packetData, uint16_t length, bool append_crc);
    void sendData(volatile uint8_t* packetData, uint8_t length);
    uint8_t recvData(volatile uint8_t* data, uint8_t length);
};

/**
 * @brief Class for ModbusRTU server
 */
class ModbusRTUServer: public ModbusRTU{

    private:
    #if !USE_EXTERNAL_INPUT_REGISTER_BUFFER
    uint16_t inputRegisters[INPUT_REGISTER_NUM] = {0};
    #else
    uint16_t* inputRegisters = NULL;
    #endif

    #if !USE_EXTERNAL_HOLDING_REGISTER_BUFFER
    uint16_t holdingRegisters[HOLDING_REGISTER_NUM] = {0};
    #else
    uint16_t* holdingRegisters = NULL;
    #endif

    void(*readInputRegistersEvent) (uint8_t* buffer, uint16_t bufferLen, void* ctx) = NULL;
    void* readInputRegistersEventCtx = NULL;
    void(*readHoldingRegistersEvent) (uint8_t* buffer, uint16_t bufferLen, void* ctx) = NULL;
    void* readHoldingRegistersEventCtx = NULL;
    void(*writeHoldingRegisterEvent) (uint8_t* buffer, uint16_t bufferLen, void* ctx) = NULL;
    void* writeHoldingRegisterEventCtx = NULL;

    void sendErrorResponse(volatile requestPacket* packet, uint8_t error_code);
    void readRegistersHandler(volatile requestPacket* packet);
    bool writeRegisterHandler(volatile requestPacket* packet);
    bool handleRequest(requestPacket* packet);

    public:
    
    #if USE_EXTERNAL_INPUT_REGISTER_BUFFER
    /**
     * @brief Sets custom buffer for input registers
     * @param inputs Input registers
     */
    void setInputRegistersBuffer(uint16_t* inputs){inputRegisters = inputs;}
    #endif

    #if USE_EXTERNAL_HOLDING_REGISTER_BUFFER    
    /**
     * @brief Sets custom buffer for input registers
     * @param inputs Input registers
     * @param size Length of buffer
     */
        void setHoldingRegistersBuffer(uint16_t* holding){holdingRegisters = holding;}
    #endif

    /**
     * @brief Sets event, which will be called when input registers are read (right before response is sent)
     * @param event Function pointer to event handler
     * @param ctx User-defined context, which will be passed to event handler
     */
    void setReadInputRegistersEvent(void(*event) (uint8_t* buffer, uint16_t bufferLen, void* ctx), void* ctx){
        readInputRegistersEvent = event; readInputRegistersEventCtx = ctx;}

    /**
     * @brief Sets event, which will be called when holding registers are read (right before response is sent)
     * @param event Function pointer to event handler
     * @param ctx User-defined context, which will be passed to event handler
     */
    void setReadHoldingRegistersEvent(void(*event) (uint8_t* buffer, uint16_t bufferLen, void* ctx), void* ctx){
        readHoldingRegistersEvent = event; readHoldingRegistersEventCtx = ctx;}

    /**
     * @brief Sets event, which will be called when holding register is written (right before data are written)
     * @param event Function pointer to event handler
     * @param ctx User-defined context, which will be passed to event handler
     */
    void setWriteHoldingRegisterEvent(void(*event) (uint8_t* buffer, uint16_t bufferLen, void* ctx), void* ctx){
        writeHoldingRegisterEvent = event; writeHoldingRegisterEventCtx = ctx;}

    #if !USE_CUSTOM_READ_WRITE_FUNCTIONS
    /**
     * @brief Sets ModbusRTU communication parameters. 
     * 
     * @param address Device Modbus address
     * @param baudRate Communication baud rate
     * @param serialPort HardwareSerial port to be used (default: Serial)
     * @param initialize If true, serial port will be initialized in this function (default: true)
     * @note Function sets the length of expected bytes to MODBUS_REQUEST_BASE_LENGTH
     */
    void startModbusServer(uint16_t address, uint32_t baudRate, HardwareSerial& serialPort = Serial, bool initialize = true){
        start(address, baudRate, serialPort, initialize);
        defaultSerialCtx.setTransTimeout(MODBUS_REQUEST_BASE_LENGTH);    
    }
    #else
    /**
     * @brief Sets ModbusRTU communication parameters. 
     * */
    void startModbusServer(uint16_t address){
        start(address);
    }
    #endif

    /**
     * @brief Main communication loop. Call this function periodically.
     * @return True if new data arrived (write request), false otherwise
     */
    bool communicationLoop();

    /**
     * @brief Saves data to input registers buffer
     * 
     * @param data Data to be saved
     * @param length Length of data
     * @param startAddress Start address in input registers buffer
     */
    void copyToInputRegisters(uint16_t* data, uint16_t length, uint16_t startAddress);

    /**
     * @brief Saves data to holding registers buffer
     * 
     * @param data Data to be saved
     * @param length Length of data
     * @param startAddress Start address in holding registers buffer
     */
    void copyToHoldingRegisters(uint16_t* data, uint16_t length, uint16_t startAddress);

    /**
     * @brief Reads data from holding registers buffer
     * 
     * @param data Buffer where data will be stored
     * @param length Length of data to be read
     * @param startAddress Start address in holding registers buffer
     */
    void copyFromHoldingRegisters(uint16_t* data, uint16_t length, uint16_t startAddress);

    /**
     * @brief Sets single value in input registers buffer
     * @param address Address of register
     * @param value Value to be set
     */
    void setInputValue(uint16_t address, uint16_t value){
        if (address < INPUT_REGISTER_NUM) inputRegisters[address] = value;
    }

    /**
     * @brief Sets single value in holding registers buffer
     * @param address Address of register
     * @param value Value to be set
     */
    void setHoldingValue(uint16_t address, uint16_t value){
        if (address < HOLDING_REGISTER_NUM) holdingRegisters[address] = value;
    }

    /**
     * @brief Gets single value from input registers buffer
     * @param address Address of register
     */
    uint16_t getHoldingValue(uint16_t address){
        if (address < HOLDING_REGISTER_NUM) return holdingRegisters[address];
        return 0;
    }
};

/**
 * @brief Class for modbus client
 */
class ModbusRTUClient: public ModbusRTU{

    private:
    int readRegisters(requestPacket* packet, uint16_t* responseBuffer, uint32_t timeout, bool allowException);
    int writeRegisters(requestPacket* packet, uint16_t* registers, uint32_t timeout, bool allowException);

    public:
    
    #if !USE_CUSTOM_READ_WRITE_FUNCTIONS
    /**
     * @brief Sets ModbusRTU communication parameters. 
     * 
     * @param address Device Modbus address
     * @param baudRate Communication baud rate
     * @param serialPort HardwareSerial port to be used (default: Serial)
     * @param initialize If true, serial port will be initialized in this function (default: true)
     */
    void startModbusClient(uint16_t address, uint32_t baudRate, HardwareSerial& serialPort = Serial, bool initialize = true){
        start(address, baudRate, serialPort, initialize);
        defaultSerialCtx.acceptException = true;
    }

    #else
    /**
     * @brief Sets ModbusRTU communication parameters. 
     * */
    void startModbusServer(uint16_t address){
        start(address);
    }
    #endif

    /**
     * @brief Executes read input registers transaction
     * @param firstRegister Address of the first register
     * @param registerNum Number of registers
     * @param registers Buffer where data will be stored
     * @param timeout Transaction timeout (in microseconds)
     * @param allowExceptions Whether treat exception as valid transaction
     * @return 0 if response were successfully received, -1 otherwise, positive exception code in case of exception
     */
    int ReadInputRegisters(uint16_t firstRegister, uint16_t registerNum, uint16_t* registers, uint32_t timeout = 200000, bool allowException = false);

    /**
     * @brief Executes read holding registers transaction
     * @param firstRegister Address of the first register
     * @param registerNum Number of registers
     * @param registers Buffer where data will be stored
     * @param timeout Transaction timeout (in microseconds)
     * @param allowExceptions Whether treat exception as valid transaction
     * @return 0 if response were successfully received, -1 otherwise, positive exception code in case of exception
     */
    int ReadHoldingRegisters(uint16_t firstRegister, uint16_t registerNum, uint16_t* registers, uint32_t timeout = 200000, bool allowException = false);

    /**
     * @brief Executes write single register transaction
     * @param firstRegister Address of the first register
     * @param data Single register data
     * @param timeout Transaction timeout (in microseconds)
     * @param allowExceptions Whether treat exception as valid transaction
     * @return 0 if response were successfully received, -1 otherwise, positive exception code in case of exception
     */
    int WriteSingleRegister(uint16_t firstRegister, uint16_t data, uint32_t timeout = 200000, bool allowException = false);

    /**
     * @brief Executes write multiple registers transaction
     * @param firstRegister Address of the first register
     * @param registerNum Number of registers
     * @param registers Buffer with registers data
     * @param timeout Transaction timeout (in microseconds)
     * @param allowExceptions Whether treat exception as valid transaction
     * @return 0 if response were successfully received, -1 otherwise, positive exception code in case of exception
     */
    int WriteMultipleRegisters(uint16_t firstRegister, uint16_t registerNum, uint16_t* registers, uint32_t timeout = 200000, bool allowException = false);

};





