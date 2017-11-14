#include "Arduino.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

int16_t gyroX, gyroY, gyroRateX, gyroRateY;
float gyroAngleX, gyroAngleY;
unsigned long currTime, prevTime=0, loopTime;

void setup() {
  mpu.initialize();
  Serial.begin(9600);
}

void loop() {
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;

  gyroX = mpu.getRotationX();
  gyroY = mpu.getRotationY();
  gyroRateX = map(gyroX, -32768, 32767, -250, 250);
  gyroRateY = map(gyroY, -32768, 32767, -250, 250);
  gyroAngleX = gyroAngleX + (float)gyroRateX*loopTime/1000;
  gyroAngleY = gyroAngleY + (float)gyroRateY*loopTime/1000;

  Serial.print("Angle X: ");
  Serial.println(gyroAngleX);
  Serial.print("Angle Y: ");
  Serial.println(gyroAngleY);
}
