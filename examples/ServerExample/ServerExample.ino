#include <Arduino.h>
#include "ModbusRTU.h"

/*This example demonstrates the usage of modbus server and data manipulation by storing mock data into registers.
Each time new data is received, built-in LED will change its state. You can test the example using simple modbus
client, for example Modbus Poll, or ClientExample from this library. 
*/

//Server object declaration
ModbusRTUServer modbus;
bool previous = false;

void setup(){
    pinMode(LED_BUILTIN, OUTPUT);

    //Set modbus server address and baud rate.
    modbus.startModbusServer(1, 115200UL);

    uint16_t inputBuffer[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    uint16_t holdingBuffer[10] = {10, 11, 12, 13, 14, 15, 16, 17, 18, 19};

    //Set some mock data into registers. You can read and write these values using modbus client.
    modbus.setMultipleHoldingRegistersValues(0, holdingBuffer, 10);
    modbus.setMultipleInputRegistersValues(0, inputBuffer, 10);

    modbus.setInputRegisterValue(10, 10);
    modbus.setHoldingRegisterValue(10, 20);
}

void loop(){

    //Call communication loop periodically.
    bool newData = modbus.communicationLoop();

    //If new data arrived, change state of built-in LED.
    if (newData != false){
        digitalWrite(LED_BUILTIN, previous);
        previous = !previous;
    }
    delay(1);
}