# ModbusRTU
## Description
Library for ModbusRTU communication protocol support which is able to run synchronously with main thread. Modbus is communication protocol based on 16-bit registers. Rtu version uses serial line (here, RTU means real-time UART).
This ModbusRtu library is ultra light, so it implements only ReadInputRegisters, ReadHoldingRegisters and WriteSingleRegister functions on server and client side, and WriteMultipleRegisters only on client side.
Due to it's lightweight nature, it does not take much computational power. Thus it can run on single-core CPUs with
relatively low frequency (i.e. Arduino Uno) without significal inpact on main program performance.

## Usage
### Minimal setup
#### Basic communication
Library uses default serial port for communication. Default settings are:
- 8 bit transmission
- 1 parity bit (even parity)
- 1 stop bit

These settings are required by ModbusRTU specification.

User should pick one of the classes - **ModbusRTUServer** or **ModbusRTUClient**,
according to their requirements. The base **ModbusRTU** class serves only as a common
interface for data transmission.

#### Basic functions - Server
To initialize Modbus server, call *startModbusServer* function with specified
device address and baud rate. If no serial port has been specified, default one
will be initialized and used for communication.

To read and write values from/to specific registers, use *setInputValue*,
*setHoldingValue* and *getHoldingValue* functions
with specified register address.

To read and write bulks of datas, use *copyToInputRegisters*, *copyToHoldingRegisters*
and *copyFromHoldingRegisters* functions. These function require buffer to copy data
from/to, number of copied registers and address of first register.

To operate modbus server, call *communicationLoop* function in main program
loop. This function returns true if new data have been received (WriteSingleRegister function), false otherwise

#### Basic functions - Client
To initialize Modbus client, call *startModbusClient* function with specified
device address and baud rate. If no serial port has been specified, default one
will be initialized and used for communication.

To read values from registers, use *readInputRegisters* and *readHoldingRegisters* functions.

To write data into registers, use *writeSingleRegister* and *writeMultipleRegisters* functions.

Optionally, user can decide whether should Modbus exception be considered as a valid response.
In that case, exception code is retained from request which caused it.
Also, user can specify timeout between request and respone (default is 200 ms).

### Additional settings
#### UART port configuration
If the device contains more serial UART communication interfaces, user can specify
which one will be used by providing it as a parameter to *startModbusServer* or
*startModbusClient* function.

Additionally, if user wants to initialize serial port manually (perhaps with custom
settings), they may set parameter *initialize* of *startModbus...* function to false. 
In this case, user is responsible for initialising the serial port before calling this function.

#### Custom buffer definition
In case user wants to use the custom register buffer for server instead of the default one, they can specify one using *setInputRegisteBuffer* function. Similar applies for holding
register (*setHoldingRegisterBuffer* function). These buffers must be able to hold unsigned
16-bit values.

Additionally *USE_EXTERNAL_INPUT_REGISTER_BUFFER*,
resp. *USE_EXTERNAL_HOLDING_REGISTER_BUFFER* must be set to **true**.
**NOTE:** This setting can also be used if only client-side library will be used. In that case, default buffers will
not be created, which will save some memory.

In case the user wants to use the default register
buffers, but with different size, they can modify *INPUT_REGISTER_NUM* and
*HOLDING_REGISTER_NUM* defines.

By default, variable length buffer for raw data will be used each time the response is constructed (in case of server) or read (in case of client). In case user wants to fix the size of this buffer, they may set *USE_FIXED_SCRATCH_BUFFER_SIZE* define to **true**.  In this case, *SCRATCH_BUFFER_SIZE* define must be set to specify response buffer size. The size must be at least 8 bytes.

#### Custom read/write functions
To use custom function to read/write from/to a serial port, use *setSerialReadFunction*
and *setSerialWriteFunction*. The custom function must accept these parameters:
- buffer where to store request/response
- length of buffer (number of bytes to send or number of bytes to read)
- context - a void pointer used to pass custom function all 
the data user decides to pass it

  The context can be anything, f.e. a structure containig serial port handler

The write function return value is void.
The read function value is number of read bytes, or 0 if nothing usable was received.

Additionally, *USE_CUSTOM_READ_WRITE_FUNCTIONS* must be set to **true**

#### Timestamp function
By default, *micros* function is called to obtain current timestamp. This can be altered by modifying GET_TIMESTAMP_US define. The timestamp function mus return the value in microseconds, in **uint32_t** format.

#### Request callbacks (server only)
Library supports user-defined callbacks which are called when specific request is received.

To set callback for ReadInputRegisters request, use *setReadInputRegistersCallback* function. This callback is called right before the response is sent.

To set callback for ReadHoldingRegisters request, use *setReadHoldingRegistersCallback* function. This callback is called right before the response is sent.

To set callback for WriteSingleRegister request, use *setWriteHoldingRegisterCallback* function. This callback is called right after the request is received, before data are stored in specified register.

Each callback function must accept these parameters:
- buffer with request/response data
- length of the buffer
- context - a void pointer used to pass custom function all the data user decides to pass it

To register function as a callback, use the corresponding *set...Callback* function. The context parameter can be used to pass any data user wants to the callback function.

### Implementation limits and further usage
Server-side library is set to accept only one request at a time. Additionally, the request must be 8 bytes long. This may be drawbacks for some use cases,
but it enables further optimization. For example, DMA can be used to transfer data from serial buffer. Also, no time is wasted by polling serial line for potential data, when size of the packet is unknown. 

There is also timeout for transaction (from the first byte to the last one), so if the data are not fully received within specified time, they are discarded.
This way, the buffer is not getiting clogged with fragments of messages.

Although library was build using Arduino framework, it is possible to use it even outside
Arduino environment (see **Custom read/write functions** and **Timestamp functions**).
