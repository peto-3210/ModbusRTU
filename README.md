# ModbusRTU
## Description
This is an implementation of ModbusRTU communication protocol, capable of running synchronously with main thread.

Modbus is widely used among industrial automation devices. Data are held and transmitted using 16-bit registers. RTU version of the protocol uses serial line (here, Rtu means real-time UART) for for data transfer.

This particular ModbusRtu library is ultra light, which means only *ReadInputRegisters*, *ReadHoldingRegisters* and *WriteSingleRegister* functions on both server and client side (and *WriteMultipleRegisters* on client side only) are implemented.
Due to it's lightweight nature, the communication does not take much computational power. This way, it can operate on single-core CPUs with relatively low frequency (i.e. Arduino Uno) without significant impact on main program performance.

## Usage
### Minimal setup
#### Basic communication
Library uses default serial port for communication. Default communication settings are:
- 8 bit transmission
- 1 parity bit (even parity)
- 1 stop bit

These settings are required by ModbusRTU specification.

You must pick one of the classes - **ModbusRTUServer** or **ModbusRTUClient**,
according to their requirements. The base **ModbusRTU** class serves only as a common
interface for data transmission.

#### Basic functions - Server
For Modbus server initialization, call *startModbusServer* function with
device address and baud rate.

For reading and writing values from/to specific registers, use *setInputRegisterValue*,
*setHoldingRegisterValue* and *getHoldingRegisterValue* methods
with specified register address.

For reading and writing bulks of data, use *setMultipleInputRegistersValues*, *setMultipleHoldingRegistersValues*
or *getMultipleHoldingRegistersValues* methods, respectively. These methods require buffer to copy data
from/to, number of copied registers and address of first register.

For modbus server operation, call *communicationLoop* method in main program
loop. This method returns true if new data have been received (WriteSingleRegister function), false otherwise
or in case of error.

#### Basic functions - Client
For Modbus client initialization, call *startModbusClient* method with
device address and baud rate.

Use *ReadInputRegisters* and *ReadHoldingRegisters* methods to read values from registers.
In each method, address of the first register, number of registers and buffer to store data to
are required.

Use *WriteSingleRegister* and *WriteMultipleRegisters* methods to write values to registers.
In each method, address of the first register and the data to be written are required.
In case of writting multiple registers, number of registers is also required.

In each method, timeout (in milliseconds) for response can be set (default value is 200).
Additionally, you can decide whether Modbus exception should be considered as a valid response (in this case, exception code is returned).

A register count outside the range allowed by the protocol (0, or more than 125 for a read
and 123 for a write) is rejected locally and nothing is transmitted; the call returns -1.

#### Exception codes returned by the server
| Code | Name | When the server sends it |
| --- | --- | --- |
| 1 | Illegal function | Function code is not one of 3, 4 or 6 |
| 2 | Illegal address | Requested range falls outside the register buffer
| 3 | Illegal data value | Register count is 0, above 125, or larger than the scratch buffer |

### Additional settings
#### Serial port configuration
If the device contains more **Serial** communication interfaces, you can specify
which one will be used, by providing it as a parameter to *startModbusServer* or
*startModbusClient* method. This can be usefull if you want to utilize multiple
instances of servers/clients on the same device.

Additionally, if you want to initialize serial port manually (perhaps with custom
settings), a parameter *initialize* of *startModbus...* method must be set to **false** (in this case, you are responsible for the serial port initialization before calling this method).

#### Custom buffer definition
You can specify the number of Input and Holding registers using **INPUT_REGISTER_NUM** and
**HOLDING_REGISTER_NUM**. Default value for both is 100.

In case you want to use custom register buffers for server instead of the default ones, macro
**USE_EXTERNAL_REGISTER_BUFFERS** must be set to **true**. 
Custom buffers must then be specified using *setInputRegistersBuffer* and *setHoldingRegistersBuffer* methods, 
**before** the server is started. 
These buffers must be able to hold **INPUT_REGISTER_NUM** and **HOLDING_REGISTER_NUM** unsigned 16-bit values.

By default, **variable length buffer** for raw data will be used each time the response is constructed (in case of server) or read (in case of client). If you want to fix the size of the buffer, **USE_FIXED_SCRATCH_BUFFER** must be set to **true**.  In this case, **SCRATCH_BUFFER_SIZE** must be set to specify response buffer size. 
**NOTE:** The buffer must be at least 8 bytes long.

#### CRC calculation
CRC calculation is done using a lookup table. If you deploy the library on a device with very limited memory, you can disable the lookup table by setting **USE_CRC_TABLE** to **false**. In this case, CRC will be calculated using bitwise operations.

If the device supports Flash memory storage, the lookup table can be stored in Flash memory instead of RAM. This can be done by setting **USE_FLASH** to **true**. In this case, you must define **FLASH_ATTR** macro to specify the Flash memory storage attribute. For example, on ESP platform, this attribute is **PROGMEM**. You must also define **FLASH_READ_WORD** macro to specify how to read a word from Flash memory. For example, on ESP platform, this macro is defined as **pgm_read_word**.

#### Custom read/write functions
To use custom function to read/write from/to a serial port, **USE_CUSTOM_READ_WRITE_FUNCTIONS** must be set to **true**
The custom functions must be specified using *setSerialReadFunction* and *setSerialWriteFunction* methods, **before** the client or server is started. The custom function must 
accept these parameters:
- buffer where to store request/response
- length of buffer (number of bytes to send or number of bytes to read)
- context - user-defined data passed to function

The context can be anything, f.e. a structure containig serial port handler

The write function return value is void.
The read function value is number of read bytes, or 0 if nothing usable was received.

**NOTE:** If custom functions for serial communication handling are used, serial port initialization must be
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
Library can handle multiple Modbus servers or clients in the same program, but each one of them must use
unique serial port.

Server-side library is set to accept only one request at a time. Additionally, the request packet must be exactly 8 bytes long (all supported requests are). This may be drawbacks for some use cases,
but it enables further optimization. For instance, DMA mechanism can be used to transfer data from serial buffer. Additionally, when the size of the packet is known in advance, no time is wasted by polling serial line for potential data. 

The reading of each packet has a timeout (from the first byte to the last one), so if the expected number of bytes is not fully received within specified time, all read bytes are discarded. Moreover, if there are some trailing bytes in receive buffer after reading finished,
the trailing bytes are flushed.
This way, the buffer is not getiting clogged with fragments of messages. 
**NOTE**: This is true only if the default functions for sending and receiving data through serial line are used. 
Otherwise, the user must implement this functionality manually in the custom read/write functions.

Although library was build using Arduino framework, it is possible to use it even outside
Arduino environment.
This might be achieved by utilizing these steps:
- modify **GET_TIMESTAMP_US** to use platform-specific function with same behaviour as *micros*
- set **USE_CUSTOM_READ_WRITE_FUNCTIONS** to **true** and provide platform-specific
functions to operate serial line using *setSerialReadFunction* and *setSerialWriteFunction* methods.

If all steps from above are applied, the library will only be used for communication packets handling, without physically 
sending/receiving any data.
Therefore, it can be used in every device with UART interface.

