/**
 * IMU emulator class interface
 */

#ifndef IMUEMU_H
#define IMUEMU_H

class ImuEmu {
    public:
        ImuEmu(); 
        int begin();
        int readAcceleration(float& x, float& y, float& z);
        int readGyroscope(float& x, float& y, float& z);
    private:
        
};

// Declare global object (to emulate Arduino LSM9DS1 library)
extern ImuEmu IMU;

#endif // IMUEMU_H