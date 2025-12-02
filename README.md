# ModbusRTU
## Description
Library for ModbusRTU communication protocol support which is able to run synchronously with main thread. Modbus is communication protocol based on 16-bit registers. Rtu version uses serial line (here, RTU means real-time UART).
This ModbusRtu library is ultra light, so it implements only ReadInputRegisters, ReadHoldingRegisters and WriteSingleRegister functions.
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

#### Basic functions
To initialize Modbus server, call *startModbusServer* function with specified
device address and baud rate. If no serial port has been specified, default one
will be initialized and used for communication.

To read and write values from/to single registers, use *readInputRegister*,
*readHoldingRegister* and *writeHoldingRegister* functions
with specificed register address.

To read and write bulks of datas, use *copyFromInputRegisters*, *copyFromHoldingRegisters*
and *copyToHoldingRegisters* functions. These function require buffer to copy data
from/to, number of copied registers and starting address.

To operate modbus server, call *communicationLoop* function in main program
loop. This function returns true if new data have been received (writeSingleRegister function), false otherwise

### Additional settings
#### UART port configuration
If the device contains more serial UART communication interfaces, user can specify
which one will be used by providing it as a parameter to *startModbusServer* function

Additionally, if user wants to initialize serial port manually (perhaps with custom
settings), they may set parameter *initialize* of *startModbusServer* function to false. 
In this case, user is responsible for initializing the serial port before calling *startModbusServer* function

#### Custom buffer definition
In case user wants to use the custom register buffer instead of the default one, they are able
to specify one using *setInputRegisteBuffer* function. Similar applies for holding
register (*setHoldingRegisterBuffer* function). These buffers must consist of unsigned
16-bit values

Additionally *USE_EXTERNAL_INPUT_REGISTER_BUFFER*,
resp. *USE_EXTERNAL_HOLDING_REGISTER_BUFFER* must be set to **true**.

In case user wants to use default register
buffers, but wants to change their size, they can do so by modifying *INPUT_REGISTER_NUM* and
*HOLDING_REGISTER_NUM* defines

By default, variable length buffer will be used each time the response is constructed. In case user wants to use fixed size response buffer, they may set *USE_FIXED_RESPONSE_BUFFER_SIZE* define to **true**.  In this case, *RESPONSE_BUFFER_SIZE* define must be set to specify response buffer size. The size must be at least 8 bytes.

#### Custom read/write functions
To use custom function to read/write from/to serial buffer, use *setSerialReadFunction*
and *setSerialWriteFunction*. The custom function must accept these parameters:
- buffer where to store request/response
- length of buffer (only for write function, length of    request is fixed)
- context - a void pointer used to pass custom function all 
the data user decides to pass it

  The context can be anything, f.e. a structure containig serial port handler, or even mock data in case of testing

Additionally, *USE_CUSTOM_READ_WRITE_FUNCTIONS* must be set to **true**

#### Request callbacks
Library supports user-defined callbacks which are called when specific request is received.

To set callback for ReadInputRegisters request, use *setReadInputRegistersCallback* function. This callback is called right before the response is sent.

To set callback for ReadHoldingRegisters request, use *setReadHoldingRegistersCallback* function. This callback is called right before the response is sent.

To set callback for WriteSingleRegister request, use *setWriteHoldingRegisterCallback* function. This callback is called right after the request is received, before data are stored in specified register.

Each callback function must accept these parameters:
- buffer with request/response data
- length of the buffer
- context - a void pointer used to pass custom function all the data user decides to pass it

To register function as a callback, use the corresponding *set...Callback* function. The context parameter can be used to pass any data user wants to the callback function.

### Implementation limits
Library is set to accept only one request at a time. Additionally, the request must be 8 bytes long. This may be drawbacks for some use cases,
but it enables further optimization. For example, DMA can be used to transfer data from serial buffer. Also, no time is wasted by polling serial buffer for new data. 

There is also timeout for reading, so if the request is not fully received within specified time, it is discarded.
This way, the buffer is not getiting clogged with fragments of messages.
