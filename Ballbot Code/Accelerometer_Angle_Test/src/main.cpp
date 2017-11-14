#include "Arduino.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

MPU6050 mpu;

int16_t accX, accY, accZ;
float accAngleX, accAngleY;

void setup() {
  mpu.initialize();
  Serial.begin(9600);
}

void loop() {
  accZ = mpu.getAccelerationZ();
  accY = mpu.getAccelerationY();
  accX = mpu.getAccelerationX();

  accAngleX = atan2(accX, accZ)*RAD_TO_DEG;
  accAngleY = atan2(accY, accZ)*RAD_TO_DEG;

  Serial.print("Angle X: ");
  Serial.println(accAngleX);
  Serial.print("Angle Y: ");
  Serial.println(accAngleY);
}
