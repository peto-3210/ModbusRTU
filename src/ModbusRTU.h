#ifndef MODBUSRTU_H
#define MODBUSRTU_H

#include <Arduino.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

//Adjust if necessary
#define INPUT_REGISTER_NUM 100
#define HOLDING_REGISTER_NUM 100
#define USE_EXTERNAL_REGISTER_BUFFERS false
#define USE_CUSTOM_READ_WRITE_FUNCTIONS false
#define USE_FIXED_SCRATCH_BUFFER false
#define SCRATCH_BUFFER_SIZE 256
#define GET_TIMESTAMP_US micros
#define USE_CRC_TABLE true
#define USE_FLASH false
//Uncomment if using flash
//#define FLASH_ATTR PROGMEM
//#define FLASH_READ_WORD pgm_read_word

#if USE_FIXED_SCRATCH_BUFFER && SCRATCH_BUFFER_SIZE < 8
    #error "Scratch buffer must be at least 8 bytes long!"
#endif

#if USE_FLASH
    #ifndef FLASH_ATTR
        #error "Flash attribute is not defined!"
    #endif
    #ifndef FLASH_READ_WORD
        #error "Function for flash reading is not defined!"
    #endif
#endif

//ModbusRTU defines (do not change)
#define MODBUS_REQUEST_BASE_LENGTH 6
#define MODBUS_RESPONSE_BASE_LENGTH 3
#define CRC_LEN 2

#define FC_READ_HOLDING_REGISTERS 3
#define FC_READ_INPUT_REGISTERS 4
#define FC_WRITE_SINGLE_REGISTER 6
#define FC_WRITE_MULTIPLE_REGISTERS 16

#define EX_ILLEGAL_FUNCTION 1
#define EX_ILLEGAL_ADDRESS 2
#define EX_ILLEGAL_DATA_VALUE 3
//#define EX_SERVER_BUSY 6
#define MAX_READ_REGISTER_COUNT 125
#define MAX_WRITE_REGISTER_COUNT 123

/**
 * @brief Puts a 16-bit value into a buffer of bytes, low byte first.
 *
 */
static inline void put_16bit_into_byte_buffer(volatile uint8_t* buffer, uint16_t offset, uint16_t value){
    buffer[offset] = (uint8_t)(value & 0xff);
    buffer[offset + 1] = (uint8_t)((value & 0xff00) >> 8);
}

/**
 * @brief Gets a 16-bit value from a buffer of bytes, low byte first.
 */
static inline uint16_t get_16bit_from_byte_buffer(const volatile uint8_t* buffer, uint16_t offset){
    return (uint16_t)(((uint16_t)buffer[offset + 1] << 8) | (uint16_t)buffer[offset]);
}

/**
 * @brief Swaps endianity of a 16-bit value.
 */
static inline uint16_t endianity_swap_16bit(uint16_t value){
    return (uint16_t)(((value & 0x00ff) << 8) | ((value & 0xff00) >> 8));
}

/**
 * @brief Structure for received modbus packet
 */
typedef union {
    uint8_t raw_data[MODBUS_REQUEST_BASE_LENGTH + CRC_LEN];
    struct {
        uint8_t address;
        uint8_t function_code;
        uint16_t first_register;
        union {
            uint16_t register_count;
            uint16_t single_register_data;
        };
        //Last member holds CRC data
        uint16_t crc;
    };
} requestPacket;
static_assert(sizeof(requestPacket) == 8);

#if !USE_CUSTOM_READ_WRITE_FUNCTIONS
    /**
     * @brief Default context structure for read and write function
     */
    typedef struct {
        HardwareSerial* serial;
        uint32_t byteTransTime;
        uint32_t transactionTimeout;
        uint32_t responseTimeout;
        bool acceptException; //If true, a 5-byte long transaction is also considered valid, for it may be a Modbus exception response

        /**
         * @brief Sets transaction timeout according to number of expected bytes (excluding CRC)
         */
        void setTransTimeout(uint16_t byteNum){
            transactionTimeout = byteTransTime * (uint32_t)(byteNum + CRC_LEN + 5); //Add some extra time, just in case
        }

        /**
         * @brief Sets timeout for response. Zero means "do not wait": the line is
         * polled once and the call returns immediately if nothing has arrived.
         */
        void SetRespTimeout(uint32_t timeout){
            responseTimeout = timeout;
        }
    } SerialCtx;

    extern uint16_t defaultSerialReadFunction(char* buffer, uint16_t length, SerialCtx* ctx);
    extern void defaultSerialWriteFunction(const char* buffer, uint16_t length, SerialCtx* ctx);
#endif


/**
 * @brief Default class
 */
class ModbusRTU{
    protected:
    uint8_t deviceAddress = 0;

    #if USE_FIXED_SCRATCH_BUFFER
        uint8_t scratchBuffer[SCRATCH_BUFFER_SIZE];
    #endif

    #if !USE_CUSTOM_READ_WRITE_FUNCTIONS
    SerialCtx defaultSerialCtx{NULL, 0, 0, 0, false};
    SerialCtx* serialReadCtx = &defaultSerialCtx;
    SerialCtx* serialWriteCtx = &defaultSerialCtx;
    uint16_t (*serialReadFunction)(char* buffer, uint16_t length, SerialCtx* ctx) = defaultSerialReadFunction;
    void (*serialWriteFunction)(const char* buffer, uint16_t length, SerialCtx* ctx) = defaultSerialWriteFunction;

    #else
    void* serialReadCtx = NULL;
    void* serialWriteCtx = NULL;
    int (*serialReadFunction)(char* buffer, uint16_t length, void* ctx) = NULL;
    void (*serialWriteFunction)(const char* buffer, uint16_t length, void* ctx) = NULL;
    #endif

    protected:
    ModbusRTU(){}

    public:
    /* serialReadCtx / serialWriteCtx point at this object's own defaultSerialCtx,
     * so a copy would silently keep using the original's context. */
    ModbusRTU(const ModbusRTU&) = delete;
    ModbusRTU& operator=(const ModbusRTU&) = delete;

    protected:
    #if USE_CUSTOM_READ_WRITE_FUNCTIONS
    public:
    /**
     * @brief Sets custom serial read function
     *
     * This function must accept three parameters: buffer, where data will be stored,
     * number of bytes to read and pointer to context (serial port object, or any
     * user-defined data, passed to function)
     *
     * Function must return number of read bytes if data was read successfully, 0 otherwise
     *
     * @param readFunction Custom read function
     * @param readCtx User-defined context, which will be passed to read function
     */
    void setSerialReadFunction(int (*readFunction)(char* buffer, uint16_t length, void* ctx), void* readCtx){
        serialReadFunction = readFunction;
        serialReadCtx = readCtx;
    }

    /**
     * @brief Sets custom serial write function
     *
     * This function must accept three parameters: buffer, which holds data to be sent,
     * length of data to be sent and pointer to context (serial port object, or any user-defined data, passed to function)
     *
     * Function must be of type void.
     *
     * @param writeFunction Custom write function
     * @param writeCtx User-defined context, which will be passed to write function
     */
    void setSerialWriteFunction(void (*writeFunction)(const char* buffer, uint16_t length,  void* ctx), void* writeCtx){
        serialWriteFunction = writeFunction;
        serialWriteCtx = writeCtx;
    }

    protected:
    void start(uint8_t address);

    #else
    void start(uint8_t address, uint32_t baudRate, HardwareSerial& serialPort, bool initialize);
    #endif

    bool calculateCRC(volatile uint8_t* packetData, uint16_t length, bool append_crc);
    void sendData(volatile uint8_t* packetData, uint16_t length);
    uint16_t recvData(volatile uint8_t* data, uint16_t length);
};

/**
 * @brief Class for ModbusRTU server
 */
class ModbusRTUServer: public ModbusRTU{

    private:
    #if !USE_EXTERNAL_REGISTER_BUFFERS
    uint16_t inputRegisters[INPUT_REGISTER_NUM] = {0};
    uint16_t holdingRegisters[HOLDING_REGISTER_NUM] = {0};
    #else
    uint16_t* inputRegisters = NULL;
    uint16_t* holdingRegisters = NULL;
    #endif

    void(*readInputRegistersCallback) (uint8_t* buffer, uint16_t bufferLen, void* ctx) = NULL;
    void* readInputRegistersCallbackCtx = NULL;
    void(*readHoldingRegistersCallback) (uint8_t* buffer, uint16_t bufferLen, void* ctx) = NULL;
    void* readHoldingRegistersCallbackCtx = NULL;
    void(*writeHoldingRegisterCallback) (uint8_t* buffer, uint16_t bufferLen, void* ctx) = NULL;
    void* writeHoldingRegisterCallbackCtx = NULL;

    void sendErrorResponse(volatile requestPacket* packet, uint8_t error_code);
    void readRegistersHandler(volatile requestPacket* packet);
    bool writeRegisterHandler(volatile requestPacket* packet);
    bool handleRequest(requestPacket* packet);

    public:

    #if USE_EXTERNAL_REGISTER_BUFFERS
    /**
     * @brief Sets custom buffer for input registers
     * @param inputs Input registers buffer
     */
    void setInputRegistersBuffer(uint16_t* inputs){inputRegisters = inputs;}

    /**
     * @brief Sets custom buffer for holding registers
     * @param holding Holding registers buffer
     */
    void setHoldingRegistersBuffer(uint16_t* holding){holdingRegisters = holding;}
    #endif

    /**
     * @brief Sets custom function, called when input registers are read (right before response is sent)
     *
     * The function must accept three parameters: buffer with response data, length of data
     * in bytes and user-defined context
     *
     * @param func Callback function
     * @param ctx User-defined context passed to callback function
     */
    void setReadInputRegistersCallback(void(*func) (uint8_t* buffer, uint16_t bufferLen, void* ctx), void* ctx){
        readInputRegistersCallback = func; readInputRegistersCallbackCtx = ctx;}

    /**
     * @brief Sets custom function, called when holding registers are read (right before response is sent)
     *
     * The function must accept three parameters: buffer with response data, length of data
     * in bytes and user-defined context
     * @param func Callback function
     * @param ctx User-defined context passed to callback function
     */
    void setReadHoldingRegistersCallback(void(*func) (uint8_t* buffer, uint16_t bufferLen, void* ctx), void* ctx){
        readHoldingRegistersCallback = func; readHoldingRegistersCallbackCtx = ctx;}

    /**
     * @brief Sets custom function, called when holding register is written (right before data are written)
     *
     * The function must accept three parameters: buffer with request data, length of data
     * in bytes and user-defined context
     * @param func Callback function
     * @param ctx User-defined context passed to callback function
     */
    void setWriteHoldingRegisterCallback(void(*func) (uint8_t* buffer, uint16_t bufferLen, void* ctx), void* ctx){
        writeHoldingRegisterCallback = func; writeHoldingRegisterCallbackCtx = ctx;}

    #if !USE_CUSTOM_READ_WRITE_FUNCTIONS
    /**
     * @brief Sets ModbusRTU communication parameters.
     *
     * @param address Device address
     * @param baudRate Communication baud rate
     * @param serialPort Serial port to be used (default: Serial)
     * @param initialize Set to false if serial port is being initialized manually,
     * outside of this function (default: true)
     */
    void startModbusServer(uint8_t address, uint32_t baudRate, HardwareSerial& serialPort = Serial, bool initialize = true){
        #if USE_EXTERNAL_REGISTER_BUFFERS
            if (inputRegisters == NULL || holdingRegisters == NULL){
                //Stop execution
                while(true);
            }
        #endif

        start(address, baudRate, serialPort, initialize);
        defaultSerialCtx.setTransTimeout(MODBUS_REQUEST_BASE_LENGTH);
        //The server polls the line without blocking the main program.
        defaultSerialCtx.SetRespTimeout(0);
        //A request is always 8 bytes; a 5-byte frame is never a valid request.
        defaultSerialCtx.acceptException = false;
    }
    #else
    /**
     * @brief Sets ModbusRTU communication parameters.
     * */
    void startModbusServer(uint8_t address){
        start(address);
    }
    #endif

    /**
     * @brief Main communication loop. Call this function periodically.
     * @return True if new data arrived (via WriteSingleRegister function), false otherwise or in case of error
     */
    bool communicationLoop();

    /**
     * @brief Sets value of multiple input registers
     *
     * @param startAddress Address of the first input register
     * @param data Data to be set
     * @param length Length of data
     */
    void setMultipleInputRegistersValues(uint16_t startAddress, uint16_t* data, uint16_t length);

    /**
     * @brief Sets value of multiple holding registers
     *
     * @param startAddress Address of the first holding register
     * @param data Data to be set
     * @param length Length of data
     */
    void setMultipleHoldingRegistersValues(uint16_t startAddress, uint16_t* data, uint16_t length);

    /**
     * @brief Reads data from holding registers buffer
     *
     * @param startAddress Address of the first holding register
     * @param data Buffer for read data
     * @param length Length of data to be read
     */
    void getMultipleHoldingRegistersValues(uint16_t startAddress, uint16_t* data, uint16_t length);

    /**
     * @brief Sets value of single input register
     * @param address Address of register
     * @param value Value to be set
     */
    void setInputRegisterValue(uint16_t address, uint16_t value){
        if (address < INPUT_REGISTER_NUM) inputRegisters[address] = value;
    }

    /**
     * @brief Sets value of single holding register
     * @param address Address of register
     * @param value Value to be set
     */
    void setHoldingRegisterValue(uint16_t address, uint16_t value){
        if (address < HOLDING_REGISTER_NUM) holdingRegisters[address] = value;
    }

    /**
     * @brief Gets value of single holding register
     * @param address Address of register
     */
    uint16_t getHoldingRegisterValue(uint16_t address){
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
     * @param serialPort Serial port to be used (default: Serial)
     * @param initialize Set to false if serial port is being initialized manually,
     * outside of this function (default: true)
     */
    void startModbusClient(uint8_t address, uint32_t baudRate, HardwareSerial& serialPort = Serial, bool initialize = true){
        start(address, baudRate, serialPort, initialize);
        defaultSerialCtx.acceptException = true;
    }

    #else
    /**
     * @brief Sets ModbusRTU communication parameters.
     * */
    void startModbusClient(uint8_t address){
        start(address);
    }
    #endif

    /**
     * @brief Executes read input registers transaction
     * @param firstRegister Address of the first register
     * @param registerNum Number of registers (1 .. 125)
     * @param registers Buffer to store input registers data
     * @param timeout Transaction timeout (in milliseconds, default: 200)
     * @param allowException Whether treat exception as valid transaction (default: false)
     * @return 0 if response were successfully received, -1 otherwise, positive exception code in case of exception
     */
    int ReadInputRegisters(uint16_t firstRegister, uint16_t registerNum, uint16_t* registers, uint32_t timeout = 200, bool allowException = false);

    /**
     * @brief Executes read holding registers transaction
     * @param firstRegister Address of the first register
     * @param registerNum Number of registers (1 .. 125)
     * @param registers Buffer to store holding registers data
     * @param timeout Transaction timeout (in milliseconds, default: 200)
     * @param allowException Whether treat exception as valid transaction (default: false)
     * @return 0 if response were successfully received, -1 otherwise, positive exception code in case of exception
     */
    int ReadHoldingRegisters(uint16_t firstRegister, uint16_t registerNum, uint16_t* registers, uint32_t timeout = 200, bool allowException = false);

    /**
     * @brief Executes write single register transaction
     * @param firstRegister Address of the register
     * @param data Single register data
     * @param timeout Transaction timeout (in milliseconds, default: 200)
     * @param allowException Whether treat exception as valid transaction (default: false)
     * @return 0 if response were successfully received, -1 otherwise, positive exception code in case of exception
     */
    int WriteSingleRegister(uint16_t firstRegister, uint16_t data, uint32_t timeout = 200, bool allowException = false);

    /**
     * @brief Executes write multiple registers transaction
     * @param firstRegister Address of the first register
     * @param registerNum Number of registers (1 .. 123)
     * @param registers Buffer with registers data
     * @param timeout Transaction timeout (in milliseconds, default: 200)
     * @param allowException Whether treat exception as valid transaction (default: false)
     * @return 0 if response were successfully received, -1 otherwise, positive exception code in case of exception
     */
    int WriteMultipleRegisters(uint16_t firstRegister, uint16_t registerNum, uint16_t* registers, uint32_t timeout = 200, bool allowException = false);

};

#endif 
