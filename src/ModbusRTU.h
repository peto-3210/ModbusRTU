#include <Arduino.h>

/*Modbus is implemented as non-inverted UART with even parity and 1 stop bit. Only
ReadInputRegisters, ReadHoldingRegisters and WriteSingleRegister functions are implemented, so the
standard request packet should consist of 6 bytes + CRC (2 bytes). Protocol data, such as 
number of registers and first register address are transmitted in big endian, payload (and CRC)
is transmitted "as is" (little endian).
*/

#define BAUD_RATE 115200 //change
#define CONFIG SERIAL_8E1

//ModbusRTU variables
#define MY_ADDRESS 1 //change
#define MODBUS_REQUEST_BASE_LENGTH 6
#define MODBUS_READ_RESPONSE_BASE_LEN 3
#define CRC_LEN 2
#define SINGLE_READ_RESPONSE_LEN 5

#define FC_READ_HOLDING_REGISTERS 3
#define FC_READ_INPUT_REGISTERS 4
#define FC_WRITE_SINGLE_REGISTER 6
//#define FC_WRITE_MULTIPLE_REGISTERS 16

#define EX_ILLEGAL_FUNCTION 1
#define EX_ILLEGAL_ADDRESS 2
//#define EX_SERVER_BUSY 6

uint16_t* INPUT_REGISTERS;
uint16_t INPUT_REGISTERS_NUM = 0;
uint16_t* COMMAND_REGISTERS;
uint16_t COMMAND_REGISTERS_NUM = 0;


//Used to put 16-bit value into buffer of bytes
#define put_16bit_into_byte_buffer(buffer, offset, value) {(buffer)[(offset) + 1] = ((value) & 0xff00) >> 8; (buffer)[(offset)] = (value) & 0xff;}

//Used to get 16-bit value from buffer of bytes
#define get_16bit_from_byte_buffer(buffer, offset) (((uint16_t)((buffer)[(offset) + 1]) << 8) | (buffer)[(offset)])

//Used to swap endianity
#define endianity_swap_16bit(value) ((uint16_t)(((value) & 0xff) << 8) | (((value) & 0xff00) >> 8))

//Main functions
/**
 * @brief Initializes UART for modbus
 */
void initModbus();

/**
 * @brief Main loop for communication.
 * @return New command, -1 if none has arrived
 */
int16_t communicationLoop();

/**
 * @brief Sets buffer for input registers
 * @param inputs Input registers
 * @param size Length of buffer
 */
void SetInputRegistersBuffer(uint16_t* inputs, uint16_t size){
    INPUT_REGISTERS = inputs;
    INPUT_REGISTERS_NUM = size;
}

/**
 * @brief Sets buffer for input registers
 * @param inputs Input registers
 * @param size Length of buffer
 */
void SetCommandRegistersBuffer(uint16_t* inputs, uint16_t size){
    COMMAND_REGISTERS = inputs;
    COMMAND_REGISTERS_NUM = size;
}






