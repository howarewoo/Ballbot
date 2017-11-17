/*
   Basic_I2C.cpp : Displays basic functionaliy of MPU9250 using I^2 on Raspberry Pi.

   Copyright (c) 2016 Simon D. Levy

   Adapted from https://github.com/bolderflight/MPU9250/blob/master/examples/Basic_I2C/Basic_I2C.ino

   Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
   and associated documentation files (the "Software"), to deal in the Software without restriction, 
   including without limitation the rights to use, copy, modify, merge, publish, distribute, 
   sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
   furnished to do so, subject to the following conditions:
   The above copyright notice and this permission notice shall be included in all copies or 
   substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
   BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
   NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
   DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "MPU9250.h"
#include <stdio.h>
#include <wiringPi.h>

// an MPU9250 object with its I2C address 
// of 0x68 (ADDR to GRND) and on Teensy bus 0
MPU9250 IMU(0x68);

float ax, ay, az, gx, gy, gz, hx, hy, hz, t;
int beginStatus;

static void printData(){

  // print the data
  printf("%6.6f\t", ax);
  printf("%6.6f\t", ay);
  printf("%6.6f\t", az);

  printf("%6.6f\t", gx);
  printf("%6.6f\t", gy);
  printf("%6.6f\t", gz);

  printf("%6.6f\t", hx);
  printf("%6.6f\t", hy);
  printf("%6.6f\t", hz);

  printf("%6.6f\n", t);
}

static void setup() {

  // start communication with IMU and 
  // set the accelerometer and gyro ranges.
  // ACCELEROMETER 2G 4G 8G 16G
  // GYRO 250DPS 500DPS 1000DPS 2000DPS
  beginStatus = IMU.begin(ACCEL_RANGE_4G,GYRO_RANGE_250DPS);
}

static void loop() {
  if(beginStatus < 0) {
    delay(1000);
    fprintf(stderr, "IMU initialization unsuccessful\n");
    fprintf(stderr, "Check IMU wiring or try cycling power\n");
    delay(10000);
  }
  else{
    /* get the individual data sources */
    /* This approach is only recommended if you only
     *  would like the specified data source (i.e. only
     *  want accel data) since multiple data sources
     *  would have a time skew between them.
     */
    // get the accelerometer data (m/s/s)
    IMU.getAccel(&ax, &ay, &az);
  
    // get the gyro data (rad/s)
    IMU.getGyro(&gx, &gy, &gz);
  
    // get the magnetometer data (uT)
    IMU.getMag(&hx, &hy, &hz);
  
    // get the temperature data (C)
    IMU.getTemp(&t);
  
    // print the data
    printData();
  
    // delay a frame
    delay(50);
  
    /* get multiple data sources */
    /* In this approach we get data from multiple data
     *  sources (i.e. both gyro and accel). This is 
     *  the recommended approach since there is no time
     *  skew between sources - they are all synced.
     *  Demonstrated are:
     *  1. getMotion6: accel + gyro
     *  2. getMotion7: accel + gyro + temp
     *  3. getMotion9: accel + gyro + mag
     *  4. getMotion10: accel + gyro + mag + temp
     */
  
     /* getMotion6 */
    // get both the accel (m/s/s) and gyro (rad/s) data
    IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
    // get the magnetometer data (uT)
    IMU.getMag(&hx, &hy, &hz);
  
    // get the temperature data (C)
    IMU.getTemp(&t);
  
    // print the data
    printData();
  
    // delay a frame
    delay(50);
  
    /* getMotion7 */
    // get the accel (m/s/s), gyro (rad/s), and temperature (C) data
    IMU.getMotion7(&ax, &ay, &az, &gx, &gy, &gz, &t);
    
    // get the magnetometer data (uT)
    IMU.getMag(&hx, &hy, &hz);
  
    // print the data
    printData();
  
    // delay a frame
    delay(50);
  
    /* getMotion9 */
    // get the accel (m/s/s), gyro (rad/s), and magnetometer (uT) data
    IMU.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &hx, &hy, &hz);
  
    // get the temperature data (C)
    IMU.getTemp(&t);
  
    // print the data
    printData();
  
    // delay a frame
    delay(50);
  
    // get the accel (m/s/s), gyro (rad/s), and magnetometer (uT), and temperature (C) data
    IMU.getMotion10(&ax, &ay, &az, &gx, &gy, &gz, &hx, &hy, &hz, &t);
  
    // print the data
    printData();
  
    // delay a frame
    delay(50);
  }
}

int main(int argc, char ** argv) {

    setup();

    while (true) {
        loop();
    }
}
