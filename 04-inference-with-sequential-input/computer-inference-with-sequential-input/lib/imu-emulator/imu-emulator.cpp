/**
 * IMU emulator class definition
 */

#include "imu-emulator.h"

// Global ImuEmu object (to emulate the Arduino LSM9DS1 library)
ImuEmu IMU;

// Constructor
ImuEmu::ImuEmu() {

}

// Blank begin that does nothing
int ImuEmu::begin() {
    return 1;
}

// Read acceleration from CSV file
int ImuEmu::readAcceleration(float& x, float& y, float& z) {
    x = 1;
    y = 2;
    z = 3;
    
    return 1;
}

// Read gyroscope from CSV file
int ImuEmu::readGyroscope(float& x, float& y, float& z) {
    x = 10;
    y = 20;
    z = 30;

    return 1;
}