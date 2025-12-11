#include <Arduino.h>
#include "ModbusRTU.h"

ModbusRTUClient modbus;
bool previous = false;

void setup(){
    modbus.startModbusClient(1, 115200UL);
}

void loop(){
    uint16_t buffer1[3] = {0};
    modbus.ReadInputRegisters(0, 3, buffer1);
    modbus.ReadInputRegisters(4, 3, buffer1, 200000, true);
    uint16_t buffer2[3] = {1, 2, 3};
    modbus.WriteMultipleRegisters(0, 3, buffer2);
    modbus.WriteSingleRegister(1, 4);
    modbus.ReadHoldingRegisters(0, 3, buffer1);
}