# ModbusRTU
## Description
This is an implementation of ModbusRTU communication protocol, which is capable of running synchronously with main thread.

Modbus is widely used among industrial automation devices. The data are held and transmitted using 16-bit registers. RTU version of the protocol uses serial line (here, Rtu means real-time UART) for for data transfer.

This ModbusRtu library is ultra light, which means only ReadInputRegisters, ReadHoldingRegisters and WriteSingleRegister functions on both server and client side, and WriteMultipleRegisters only on client side are implemented.
Due to it's lightweight nature, the communication does not take much computational power. Therefore, it can operate on single-core CPUs with relatively low frequency (i.e. Arduino Uno) without significal inpact on main program performance.

## Usage
### Minimal setup
#### Basic communication
Library uses default serial port for communication. Default communication settings are:
- 8 bit transmission
- 1 parity bit (even parity)
- 1 stop bit

These settings are required by ModbusRTU specification.

User must pick one of the classes - **ModbusRTUServer** or **ModbusRTUClient**,
according to their requirements. The base **ModbusRTU** class serves only as a common
interface for data transmission.

#### Basic functions - Server
To initialize Modbus server, call *startModbusServer* function with
device address and baud rate.

To read and write values from/to specific registers, use *setInputRegisterValue*,
*setHoldingRegisterValue* and *getHoldingRegisterValue* methods
with specified register address.

To read and write bulks of data, use *setMultipleInputRegistersValue*, *setMultipleHoldingRegistersValue*
or *getMultipleHoldingRegistersValue* methods, respectively. These methods require buffer to copy data
from/to, number of copied registers and address of first register.

To operate modbus server, call *communicationLoop* method in main program
loop. This method returns true if new data have been received (via WriteSingleRegister function), false otherwise
or in case of error.

#### Basic functions - Client
To initialize Modbus client, call *startModbusClient* method with specified
device address and baud rate.

To read values from registers, use *readInputRegisters* and *readHoldingRegisters* methods.
In each method, address of the first register, number of registers and buffer to store data to
are required.

To write data into registers, use *writeSingleRegister* and *writeMultipleRegisters* methods.
In each method, address of the first register and the data to be written are required.
In case of writting multiple registers, number of registers is also required.

In each method, timeout (in milliseconds) for response can be set (default value is 200).
Additionally, user can decide whether Modbus exception should be considered as a valid response.
In that case, exception code is returned by method which caused it.

### Additional settings
#### Serial port configuration
If the device contains more Serial communication interfaces, user can specify
which one will be used by providing it as a parameter to *startModbusServer* or
*startModbusClient* method. This can be usefull if user wants to utilize multiple
instances of servers/clients on the same device.

Additionally, if user wants to initialize serial port manually (perhaps with custom
settings), a parameter *initialize* of *startModbus...* method must be set to **false**. 
In this case, user is responsible for the serial port initialization before calling this method.

#### Custom buffer definition
User can specify the number of Input and Holding registers using **INPUT_REGISTER_NUM** and
**HOLDING_REGISTER_NUM**. Default value for both is 100.

In case user wants to use the custom register buffers for server instead of the default ones, macro
**USE_EXTERNAL_REGISTER_BUFFERS** must be set to **true**. 
Custom buffers must then be specified using *setInputRegisterBuffer* and *setHoldingRegisterBuffer* methods. 
These buffers must be able to hold **INPUT_REGISTER_NUM** and **HOLDING_REGISTER_NUM** unsigned 16-bit values.

**NOTE:** **USE_EXTERNAL_REGISTER_BUFFERS** may also be set to **true** if only client-side library will be used. 
In that case, default buffers will not be created, which will save some memory.

By default, variable length buffer for raw data will be used each time the response is constructed (in case of server) or read (in case of client). If user wants to fix the size of this buffer, **USE_FIXED_SCRATCH_BUFFER_SIZE** must be set to **true**.  In this case, **SCRATCH_BUFFER_SIZE** must be set to specify response buffer size. 
**NOTE:** The buffer must be at least 8 bytes long.

#### Custom read/write functions
To use custom function to read/write from/to a serial port, **USE_CUSTOM_READ_WRITE_FUNCTIONS** must be set to **true**
The custom functions must be specified using *setSerialReadFunction* and *setSerialWriteFunction* methods. The custom function must 
accept these parameters:
- buffer where to store request/response
- length of buffer (number of bytes to send or number of bytes to read)
- context - user-defined data passed to function

The context can be anything, f.e. a structure containig serial port handler

The write function return value is void.
The read function value is number of read bytes, or 0 if nothing usable was received.

**NOTE:** If custom functions to handle serial communication are used, serial port initialization must be
done manually by user.

#### Timestamp function
By default, *micros* function is called to obtain current timestamp. This can be altered by modifying **GET_TIMESTAMP_US**. The timestamp function must return the current timestamp value in microseconds, in **uint32_t** format.

#### Request callbacks (server only)
Library supports user-defined callbacks, which are called when specific request is received.

To set callback for ReadInputRegisters request, use *setReadInputRegistersCallback* method. This callback is called right before the response is sent.

To set callback for ReadHoldingRegisters request, use *setReadHoldingRegistersCallback* method. This callback is called right before the response is sent.

To set callback for WriteSingleRegister request, use *setWriteHoldingRegisterCallback* method. This callback is called right after the request is received, before data are stored in specified register.

Each callback function must accept these parameters:
- buffer with request/response data
- length of the buffer
- context - user-defined data passed to function

### Implementation limits and further usage
Library can handle multiple Modbus servers or clients in same program, but each one of them must use
unique serial port. If the device has multiple serial ports, the custom one must be specified when
calling *startModbusServer* or *startModbusClient* method.

Server-side library is set to accept only one request at a time. Additionally, the request packet must be exactly 8 bytes long. This may be drawbacks for some use cases,
but it enables further optimization. For instance, DMA mechanism can be used to transfer data from serial buffer. Also, when the size of the packet is known in advance, no time is wasted by polling serial line for potential data. 

Each reading has a timeout (from the first byte to the last one), so if the expected number of bytes is not fully received within specified time, all read bytes are discarded. Moreover, if there are still some bytes in receive buffer after reading,
the trailing bytes are flushed.
This way, the buffer is not getiting clogged with fragments of messages. 
**NOTE**: This is true only if the default functions for sending and receiving data through serial line are used.

Although library was build using Arduino framework, it is possible to use it even outside
Arduino environment.
This might be achieved by utilizing these steps:
- modify **GET_TIMESTAMP_US** to use platform-specific function with same behaviour as *micros*
- set **USE_CUSTOM_READ_WRITE_FUNCTIONS** to **true** and provide platform-specific
functions to operate serial line using *setSerialReadFunction* and *setSerialWriteFunction* methods.
Serial port must then be initialized manually by the user.

This way, the library will only used for communication packets handling, without physically 
sending/receiving any data.
Thus, the library can be used in every device with UART interface (even on Computers).

