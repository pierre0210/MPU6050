#ifndef MPU6050Custom_h
#define MPU6050Custom_h

#include "Arduino.h"

class MPU6050Custom {
  public:
    MPU6050Custom(byte addr);
    void mpuRegister();
    void calibration();
    float* readRaw();
    float* getAngles();
    void calAcc(float* raw);
    void calGyro(float* raw);
  private:
    byte _addr;
    float _offset[6] = { 0 };
    float _result[6] = { 0 };
    float _rpy[3] = { 0, 0, 0 };
    bool _init = false;
    double _timer;
};

#endif
