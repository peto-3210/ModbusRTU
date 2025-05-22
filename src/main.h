#include <Arduino.h>
#include "ModbusRTU.h"

enum Procedures : uint8_t{
    home = 0,
    fetchCup = 1,
    exportCoffee = 2,
    calibrate = 3,
    stop = 4
};

enum Phases : uint8_t{
    homing = 0,

    waiting = 1,
    cupPick = 2,
    solenoidUnlock = 3,
    cupUnload = 4,
    solenoidLock = 5,
    liftDescend = 6,
    cupToMachine = 7,

    coffeeToLift = 8,
    liftAscend = 9,
    handDodge = 10,
    coffeeToHeliport = 11,
    coffeeTocrossroad = 12,
    handBack = 13,
    crossroadFall = 14,
    coffeeToConveyor = 15,
    crossroadRise = 16,

    calibrating = 17
};

union DiagData{
    uint8_t rawData;

    struct{
        uint8_t phaseRunning    :1;
        uint8_t calibrated      :1;
        uint8_t home            :1;
        uint8_t firstPhaseDone  :1;

        uint8_t errCupMissing   :1;
        uint8_t errCupPresent   :1;
        uint8_t errStepperLost  :1;
        uint8_t errFullConveyor :1;
    };
};

enum handPositions : long{
    handHomePosition = 1200,
    cupToMachinePosition = 14700,
    coffeeToLiftPosition = 1200,
    handDodgePosition = 14700,
    cupTocrossroadPosition = 0,
    handBackPosition = 1200
};

enum liftPositions:long{
        liftHomePosition = 0,
        cupPickupPosition = 61000,
        cupUnloadPosition = 33750,
        liftDescendPosition = 0,
        liftAscendPosition = 29000
};

static const long exportCupPosition = 14700;
