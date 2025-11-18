#include <Arduino.h>
#include "ModbusRTU.h"

ModbusRTU modbus;
bool previous = false;

void setup(){
    
    pinMode(LED_BUILTIN, OUTPUT);
    modbus.startModbusServer(1, 115200UL);
    for (uint16_t i = 0; i < 10; i++){
        modbus.copyToHoldingRegisters(&i, 1, i);
        modbus.copyToInputRegisters(&i, 1, i);
    }
}

void loop(){
    int16_t newData = modbus.communicationLoop();
    if (newData != -1){
        //New data arrived
        digitalWrite(LED_BUILTIN, previous);
        previous = !previous;
    }
    delay(1);
}