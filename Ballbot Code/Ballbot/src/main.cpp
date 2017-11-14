/* Ballbot: Omnidirectional Dynamically Stable Inverted Pendulum

Stationary Balancing Program

Written by Adam Woo, Graham Goodwin, Chloe Desjardine

This program has the main parts:
1) Reading the IMU
2)Converting the IMU readings to angles
3) PID controls to manage veloccities for each motor

Hardware used:
Arduino Mega
MPU9250

*/


//----------------------------------------------------------
// Include Libraries
//----------------------------------------------------------
#include "Arduino.h"
#include "Wire.h"
#include "TimerOne.h"
#include "I2Cdev.h"
#include "MPU9250.h"
#include "math.h"

//----------------------------------------------------------
// PID Variables
//----------------------------------------------------------
#define Kp  40
#define Kd  0.05
#define Ki  40
#define sampleTime  0.005
#define targetAngle -2.5

//----------------------------------------------------------
// MPU
//----------------------------------------------------------
#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

// Initial time
long int ti;
volatile bool intFlag=false;

// This function read Nbytes bytes from I2C device at address Address.
// Put read bytes starting at register Register in the Data array.
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data){
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  // Read Nbytes
  Wire.requestFrom(Address, Nbytes);
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data){
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

// Counter
long int cpt=0;

void callback(){
  intFlag=true;
  digitalWrite(13, digitalRead(13) ^ 1);
}

void MPUsetup(){
  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,29,0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,26,0x06);


  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);

  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS,0x0A,0x16);

   pinMode(13, OUTPUT);
  Timer1.initialize(10000);         // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt

  // Store initial time
  ti=millis();
}

/* ----------------------------------------------------------
                        Motor 1
                           /\
                         /    \
                       /        \
                     /            \
                   /                \
                 /                    \
               /                        \
             /                            \
           /                                \
          ------------------------------------
      Motor 2                              Motor 3
----------------------------------------------------------*/
const int stepsPerRevolution = 400;
const int stepPin1 = 3;
const int dirPin1 = 4;
const int stepPin2 = 5;
const int dirPin2 = 6;
const int stepPin3 = 7;
const int dirPin3 = 8;

void motorsetup(){
  // Sets the two pins as Outputs
  pinMode(stepPin1,OUTPUT);
  pinMode(dirPin1,OUTPUT);
  pinMode(stepPin2,OUTPUT);
  pinMode(dirPin2,OUTPUT);
  pinMode(stepPin3,OUTPUT);
  pinMode(dirPin3,OUTPUT);
  digitalWrite(dirPin1,HIGH); //Enables the motor to move in a particular direction
  digitalWrite(dirPin2,HIGH); //Enables the motor to move in a particular direction
  digitalWrite(dirPin3,HIGH); //Enables the motor to move in a particular direction
}



void setup(){
  // Arduino initializations
  Wire.begin();
  Serial.begin(9600);

  motorsetup();
  MPUsetup();
}

void loop(){
}
