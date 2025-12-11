#include <Arduino.h>
#include "ModbusRTU.h"

ModbusRTUServer modbus;
bool previous = false;

void setup(){
    
    pinMode(LED_BUILTIN, OUTPUT);
    modbus.startModbusServer(1, 115200UL);
    for (uint16_t i = 0; i < 10; i++){
        modbus.copyToHoldingRegisters(&i, 1, i);
        i += 10;
        modbus.copyToInputRegisters(&i, 1, i);
        i -= 10;
    }
    modbus.setInputValue(10, 10);
    modbus.setHoldingValue(10, 20);
}

void loop(){
    bool newData = modbus.communicationLoop();
    if (newData != false){
        //New data arrived
        digitalWrite(LED_BUILTIN, previous);
        previous = !previous;
    }
    delay(1);
}