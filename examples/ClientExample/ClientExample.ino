#include <Arduino.h>
#include "ModbusRTU.h"

/*This example demonstrates the usage of modbus client and data manipulation by writting and reading mock data.
You can test the example using simple modbus
client, for example pymodslave, or ServerExample from this library. 
*/

//Client object declaration
ModbusRTUClient modbus;

void setup(){
    //Set modbus server address and baud rate.
    modbus.startModbusClient(1, 115200UL);
}

void loop(){
    uint16_t buffer1[3] = {0};

    //Read input registers into buffer.
    modbus.ReadInputRegisters(0, 3, buffer1);

    //Read input registers into buffer with timeout, allowing exception as valid response.
    modbus.ReadInputRegisters(4, 3, buffer1, 200000, true);


    uint16_t buffer2[3] = {1, 2, 3};

    //Write multiple registers, then write single register.
    modbus.WriteMultipleRegisters(0, 3, buffer2);
    modbus.WriteSingleRegister(1, 4);

    //Verify that registers were written by reading them back.
    modbus.ReadHoldingRegisters(0, 3, buffer1);
}