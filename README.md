# ModbusRTU
Library for ModbusRtu, which uses serial line (UART) and is able to run synchronously with main thread.
It is ultra lightweigth, so it implements only ReadInputRegisters, ReadHoldingRegisters and WriteSingleRegister functions.
Due to it's lightweight nature, it does not take much computational power. Thus it can run on single-core CPUs with
relatively low frequency (i.e. Arduino Uno) without significal inpact on main program performance.
