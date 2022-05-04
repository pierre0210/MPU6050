#include "MPU6050Custom.h"

MPU6050Custom mpu(0x68);

void setup() {
  Serial.begin(115200);
  mpu.mpuRegister();
  mpu.calibration();
}
void loop() {
  float* rpy = mpu.getAngles();
  
  Serial.print("row:\t");
  Serial.print(rpy[0]);
  Serial.print("\tpitch:\t");
  Serial.print(rpy[1]);
  Serial.print("\tyaw:\t");
  Serial.println(rpy[2]);
  
  delay(5);
}
