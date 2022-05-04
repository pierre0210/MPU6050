#include "Arduino.h"
#include "MPU6050Custom.h"
#include <Wire.h>

MPU6050Custom::MPU6050Custom(byte addr) {
  _addr = addr;
}

void MPU6050Custom::mpuRegister() {
  // Reference: https://stackoverflow.com/questions/61754933/is-there-a-way-to-convert-raw-mpu-6050-data-to-a-0-359-rotational-data
  Wire.beginTransmission(_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(_addr);
  Wire.write(0x1A); // low pass filter
  Wire.write(0x02); // 94/98Hz low pass filter
  Wire.endTransmission();

  Wire.beginTransmission(_addr);
  Wire.write(0x1B); // gyro sensitivity
  Wire.write(0x00); // ±250°/s
  Wire.endTransmission();

  Wire.beginTransmission(_addr);
  Wire.write(0x1C); // acceleromter sensitivity
  Wire.write(0x10); // ±8g
  Wire.endTransmission();
}

void MPU6050Custom::calibration() {
  delay(100);
  int maxSamples = 2000;
  float* raw;
  
  for (int i = 0; i < maxSamples; i++) {
    raw = MPU6050Custom::readRaw();
    _offset[0] += raw[0]; // acc x
    _offset[1] += raw[1]; // acc y
    _offset[2] += raw[2]; // acc z

    _offset[3] += raw[3];
    _offset[4] += raw[4];
    _offset[5] += raw[5];

    delay(5);
  }
  
  _offset[0] /= maxSamples;
  _offset[1] /= maxSamples;
  _offset[2] /= maxSamples;

  _offset[3] /= maxSamples;
  _offset[4] /= maxSamples;
  _offset[5] /= maxSamples;

  Serial.begin(230400);
  Serial.print(_offset[0]);
  Serial.print("\t");
  Serial.print(_offset[1]);
  Serial.print("\t");
  Serial.print(_offset[2]);
  Serial.print("\t");
  Serial.print(_offset[3]);
  Serial.print("\t");
  Serial.print(_offset[4]);
  Serial.print("\t");
  Serial.print(_offset[5]);
  Serial.print("\t\n");
}

float* MPU6050Custom::readRaw() {
  static float rawData[6];
  Wire.beginTransmission(_addr);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(_addr, uint8_t(16));

  while (Wire.available() < 16);

  // acc
  rawData[0] = (Wire.read() << 8 | Wire.read());
  rawData[1] = (Wire.read() << 8 | Wire.read());
  rawData[2] = (Wire.read() << 8 | Wire.read());
  
  // temperature
  int temp = Wire.read() << 8 | Wire.read();
  //Serial.println(temp);
  // gyro
  rawData[3] = (Wire.read() << 8 | Wire.read());
  rawData[4] = (Wire.read() << 8 | Wire.read());
  rawData[5] = (Wire.read() << 8 | Wire.read());
  
  return rawData;
}

float* MPU6050Custom::getAngles() {
  float* raw = MPU6050Custom::readRaw();
  // Reference: https://mjwhite8119.github.io/Robots/mpu6050
  MPU6050Custom::calAcc(raw);
  MPU6050Custom::calGyro(raw);
  
  // Complemetary Filter
  _result[3] = _result[3] * 0.995 + _result[0] * 0.005;
  _result[4] = _result[4] * 0.995 + _result[1] * 0.005;

  _rpy[0] = _result[3];
  _rpy[1] = _result[4];
  _rpy[2] = raw[5];

  return _rpy;
}

void MPU6050Custom::calAcc(float* raw) {
  for(int i=0; i<3; i++) {
    raw[i] -= _offset[i];
    raw[i] /= 4096.0; // 8g sensitivity
  }

  _result[0] = atan2(raw[1], sqrt(pow(raw[0], 2) + pow(raw[2], 2)))*180 / PI;
  _result[1] = atan2(-raw[0], sqrt(pow(raw[1], 2) + pow(raw[2], 2)))*180 / PI;
  Serial.println(_result[1]);
}

void MPU6050Custom::calGyro(float* raw) {
  _timer = millis();
  for(int i=3; i<6; i++) {
    raw[i] -= _offset[i];
    raw[i] /= 131.0; // ±250°/s sensitivity
  }
  double dt = (double)(millis() - _timer) / 1000;
  //_timer = micros();
  Serial.println(raw[4]);
  _result[3] += raw[3]*dt;
  _result[4] += raw[4]*dt;
  //_rpy[2] += raw[5]*dt;
}
