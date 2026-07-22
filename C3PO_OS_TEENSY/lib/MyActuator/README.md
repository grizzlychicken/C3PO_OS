# MyActuator

[![Unit Tests](https://github.com/Clstroud/MyActuator/actions/workflows/run_tests.yml/badge.svg?branch=main)](https://github.com/Clstroud/MyActuator/actions/workflows/run_tests.yml)

### Setup

Add the repo to your `platformio.ini` file:
```ini
libdeps =
    https://github.com/Clstroud/MyActuator.git
```

If you want to pin a particular version tag, you can do that too:
```ini
libdeps =
    https://github.com/Clstroud/MyActuator.git#1.0.0
```


### Example

```cpp
#include <MyActuator/MyActuator.h>

using namespace MyActuator;

ActuatorBus bus(Serial1);
Actuator motorOne(0x1, ActuatorModel::X4, Protocol::V3);
Actuator motorTwo(0x2, ActuatorModel::RMD_X8, Protocol::V2);

void setup() {

    Serial.begin(115200);

    if (!bus.begin(115200)) {
        Serial.println("Failed to initialize bus!");
        while (true) {
            // oblivion
        }
    }

    if (!bus.add(motorOne)) {
        Serial.println("Failed to register actuator!");
        while (true) {
            // oblivion
        }
    }
}

void loop() {
    
    PIDParameters parameters;
    bool result = bus.getPIDParameters(motorOne, parameters, 200); // 200ms timeout to show how to customize timeout

    if (result) {
        Serial.println("Read gains successfully!");
    } else {
        Serial.println("Failed to read gains :(");
    }

    // Velocity control (with status, which can be omitted if you don't care)
    VelocityControlCommandStatus status;
    result = bus.setVelocity(motorOne, Velocity::degreesPerSecond(15.0), &status);
    // status.temp;      // 1ºC / LSB
    // status.torque;    // 0.01A/LSB
    // status.velocity;  // degrees/second
    // status.position;  // 1º/LSB with range of +/- 32767º


    // ....and more!
}

```