// Ballbot: Omnidirectional Dynamically Stable Inverted Pendulum
//
// Stationary Balancing Program
//
// Written by Adam Woo, Graham Goodwin, Chloe Desjardine
//
// This program has the main parts:
// 1) Reading the IMU
// 2)Converting the IMU readings to angles
// 3) PID controls to manage veloccities for each motor
//
// Hardware used:
// Arduino Mega
// MPU9250


//----------------------------------------------------------
// Include Libraries
//----------------------------------------------------------
#include <stdio.h>
#include <wiringPi.h>
#include <math.h>
#include <A4988.h>
#include <unistd.h>
#include <time.h>
#include <MotionSensor.h>
#include <wiringPiSPI.h>
#include <SPI.h>

//----------------------------------------------------------
// MPU settings
//----------------------------------------------------------
#define DT 20
#define G_GAIN 0.07

//----------------------------------------------------------
// PID Variables
//----------------------------------------------------------
#define Kp  40
#define Kd  0.05
#define Ki  40
#define sampleTime  0.005
#define targetAngle -2.5

//----------------------------------------------------------
// Pins and motor definitions
//----------------------------------------------------------
#define stepsPerRevolution 400
#define


#define stepPin1 3
#define dirPin1 4
#define stepPin2 5
#define dirPin2 6
#define stepPin3 7
#define dirPin3 8
#define MS1 9
#define MS2 10
#define MS3 11

// an MPU9250 object with the MPU-9250 sensor on Teensy Chip Select pin 10
MPU9250 IMU(10);
float ax, ay, az, gx, gy, gz, hx, hy, hz, t;
int beginStatus;

A4988 motor1(stepsPerRevolution, dirPin1, stepPin1, ENABLE, MS1, MS2, MS3);
A4988 motor2(stepsPerRevolution, dirPin2, stepPin2, ENABLE, MS1, MS2, MS3);
A4988 motor3(stepsPerRevolution, dirPin3, stepPin3, ENABLE, MS1, MS2, MS3);

double fallAngle, pitch;

//----------------------------------------------------------
// Setup
//----------------------------------------------------------
void setup(){
  Serial.begin(115200);
  beginStatus = IMU.begin(ACCEL_RANGE_4G,GYRO_RANGE_250DPS);
  ms_open();          // Turn on motion sensor
  stepper.begin(RPM, MICROSTEPS);



}



void readMPU(){
  if(beginStatus < 0) {
    delay(1000);
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    delay(10000);
  }
  IMU.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &hx, &hy, &hz);
  // //Convert Gyro raw to degrees per second
  // rate_gyr_x = (float) gx * G_GAIN;
  // rate_gyr_y = (float) gy * G_GAIN;
  // rate_gyr_z = (float) gz * G_GAIN;
  // //Calculate the angles from the gyro
  // long DT = endTime-startTime
  // gyroXangle+=rate_gyr_x*DT;
  // gyroYangle+=rate_gyr_y*DT;
  // gyroZangle+=rate_gyr_z*DT;
  // //Convert Accelerometer values to degrees
  // AccXangle = (float) (atan2(accRaw[1],accRaw[2])+M_PI)*RAD_TO_DEG;
  // AccYangle = (float) (atan2(accRaw[2],accRaw[0])+M_PI)*RAD_TO_DEG;
  // //Complementary Filter
  // angleX=AA*(CFangleX+rate_gyr_x*DT) +(1 - AA) * AccXangle;
  // angleY=AA*(CFangleY+rate_gyr_y*DT) +(1 - AA) * AccYangle;

  pitch = 180 * atan (ax/sqrt(ay*ay + az*az))/M_PI;
  roll  = 180 * atan (ay/sqrt(ax*ax + az*az))/M_PI;
  yaw   = 180 * atan (az/sqrt(ax*ax + az*az))/M_PI;
  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  mag_x = hx * cos_pitch + (-hy) * sin_roll * sin_pitch + (-hz) * cos_roll * sin_pitch;
  mag_y = (-hy) * cos_roll - (-hz) * sin_roll;
  MAG_Heading = atan2(-mag_y, mag_x);
  serial.print("yaw = %2.1f\tpitch = %2.1f\troll = %2.1f\ttemperature = %2.1f\tcompass = %2.1f, %2.1f, %2.1f\n", yaw, pitch, roll, temp, compass[0], compass[1], compass[2]);
}


//----------------------------------------------------------
// loop
//----------------------------------------------------------
void loop(){
  while(millis() - startInt < 20)
  {
    startInt = millis();
  }
  readMPU();
  calculations();
  moveStepper();


}

calculations(){

}

//----------------------------------------------------------
// moveStepper
//----------------------------------------------------------
void moveStepper(int stepPin, int speedDelay){
  digitalWrite(stepPin,HIGH);
  delayMicroseconds(speedDelay);
  digitalWrite(stepPin,LOW);
  delayMicroseconds(speedDelay);
}


int main(int argc, char ** argv) {

  setup();

  while (true) {
    loop();
  }
}
