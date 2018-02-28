/* Ballbot_MPU6050.ino
*
* Omnidirectional Ballbot control system with 3-axis PID controls using the
* MPU6050 IMU
*
* Written by Adam Woo
*
* MIT License
*/

#include "quaternionFilters.h"
#include "MPU6050.h"
#include "AccelStepper.h"
#include "math.h"
#include "PID_v1.h"

#define ROBOT_HEIGHT 31 //inches
#define WHEEL_RADIUS 1.625
#define MAX_SPEED 4000 //steps per second
#define sampleRate 100 //milliseconds

#define DIR1 7
#define STEP1 8
#define DIR2 9
#define STEP2 10
#define DIR3 11
#define STEP3 12
AccelStepper stepper1(1, DIR1, STEP1);
AccelStepper stepper2(1, DIR2, STEP2);
AccelStepper stepper3(1, DIR3, STEP3);

//Three PID controllers; one for each axis of rotation
//Define Variables we'll be connecting to
double CurrentAngleX, OutputSpeedX, DesiredAngleX = 0;
double CurrentAngleY, OutputSpeedY, DesiredAngleY = 0;
double CurrentAngleZ, OutputSpeedZ, DesiredAngleZ = 0;
double Kp_x=800, Kp_y=800, Kp_z=800;
double Ki_x=0.2, Ki_y=0.2, Ki_z=0.2;
double Kd_x=100, Kd_y=100, Kd_z=100;

//Specify the links and initial tuning parameters
PID xPID(&CurrentAngleX, &OutputSpeedX, &DesiredAngleX, Kp_x, Ki_x, Kd_x, P_ON_M, DIRECT);
PID yPID(&CurrentAngleY, &OutputSpeedY, &DesiredAngleY, Kp_y, Ki_y, Kd_y, P_ON_M, DIRECT);
PID zPID(&CurrentAngleZ, &OutputSpeedZ, &DesiredAngleZ, Kp_z, Ki_z, Kd_z, P_ON_M, DIRECT);

#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

// -----------------------------------------------------------------------------
// Need this to compile for some reason... No idea why. Think it has something
// to do with the way the Teensy loader works
extern "C"{
  int _getpid(){ return -1;}
  int _kill(int pid, int sig){ return -1; }
  int _write(){return -1;}
}
// -----------------------------------------------------------------------------

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int RPM = 0;

MPU6050 myIMU;

double speed1, speed2, speed3;
int timer, timerstart;

void setupIMU(){
  // Read the WHO_AM_I register, this is a good test of communication
  uint8_t c = readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x68, HEX);

  if (c == 0x68) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU6050 is online...");

    myIMU.MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(SelfTest[5],1); Serial.println("% of factory value");


    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU6050(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU6050();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

  } // if (c == 0x71)
  else{
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }
}

void readIMU() {
  // If data ready bit set, all data registers have new data
  if(readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {  // check if data ready interrupt
    readAccelData(accelCount);  // Read the x/y/z adc values
    getAres();

    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes;  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes;
    az = (float)accelCount[2]*aRes;

    readGyroData(gyroCount);  // Read the x/y/z adc values
    getGres();

    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes;
    gz = (float)gyroCount[2]*gRes;

    tempCount = readTempData();  // Read the x/y/z adc values
    temperature = ((float) tempCount) / 340. + 36.53; // Temperature in degrees Centigrade
  }

  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  //    if(lastUpdate - firstUpdate > 10000000uL) {
  //      beta = 0.041; // decrease filter gain after stabilized
  //      zeta = 0.015; // increase gyro bias drift gain after stabilized
  //    }
  // Pass gyro rate as rad/s
  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f);

  // Serial print and/or display at 0.5 s rate independent of data rates
  delt_t = millis() - count;
  if (delt_t > 500) { // update LCD once per half-second independent of read rate
    digitalWrite(blinkPin, blinkOn);
    /*
    Serial.print("ax = "); Serial.print((int)1000*ax);
    Serial.print(" ay = "); Serial.print((int)1000*ay);
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");

    Serial.print("gx = "); Serial.print( gx, 1);
    Serial.print(" gy = "); Serial.print( gy, 1);
    Serial.print(" gz = "); Serial.print( gz, 1); Serial.println(" deg/s");

    Serial.print("q0 = "); Serial.print(q[0]);
    Serial.print(" qx = "); Serial.print(q[1]);
    Serial.print(" qy = "); Serial.print(q[2]);
    Serial.print(" qz = "); Serial.println(q[3]);
    */
    // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
    // In this coordinate system, the positive z-axis is down toward Earth.
    // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
    // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
    // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
    // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
    // applied in the correct order which for this configuration is yaw, pitch, and then roll.
    // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI;
    roll  *= 180.0f / PI;

    CurrentAngleX = pitch;
    CurrentAngleY = roll;
    CurrentAngleZ = yaw;

    //    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);

    //    Serial.print("average rate = "); Serial.print(1.0f/deltat, 2); Serial.println(" Hz");

    // display.clearDisplay();
    //
    // display.setCursor(0, 0); display.print(" x   y   z  ");
    //
    // display.setCursor(0,  8); display.print((int)(1000*ax));
    // display.setCursor(24, 8); display.print((int)(1000*ay));
    // display.setCursor(48, 8); display.print((int)(1000*az));
    // display.setCursor(72, 8); display.print("mg");
    //
    // display.setCursor(0,  16); display.print((int)(gx));
    // display.setCursor(24, 16); display.print((int)(gy));
    // display.setCursor(48, 16); display.print((int)(gz));
    // display.setCursor(66, 16); display.print("o/s");
    //
    // display.setCursor(0,  32); display.print((int)(yaw));
    // display.setCursor(24, 32); display.print((int)(pitch));
    // display.setCursor(48, 32); display.print((int)(roll));
    // display.setCursor(66, 32); display.print("ypr");
    //
    // display.setCursor(0, 40); display.print("rt: "); display.print(1.0f/deltat, 2); display.print(" Hz");
    // display.display();

    blinkOn = ~blinkOn;
    count = millis();
  }
}

void setupStepper(){
  stepper1.setMaxSpeed(MAX_SPEED);
  stepper2.setMaxSpeed(MAX_SPEED);
  stepper3.setMaxSpeed(MAX_SPEED);
}

void runMotors(long steps_sec1, long steps_sec2, long steps_sec3){
  stepper1.setSpeed(steps_sec1);
  stepper2.setSpeed(steps_sec2);
  stepper3.setSpeed(steps_sec3);
  stepper1.runSpeed();
  stepper2.runSpeed();
  stepper3.runSpeed();
}

void speedCalculations(){
  speed1 = ((OutputSpeedX/2)+(0.866*OutputSpeedY)+OutputSpeedZ)/WHEEL_RADIUS;
  speed2 = ((OutputSpeedX/2)-(0.866*OutputSpeedY)+OutputSpeedZ)/WHEEL_RADIUS;
  speed3 = (OutputSpeedX+OutputSpeedZ)/WHEEL_RADIUS;
}

void setupPID(){
  xPID.SetSampleTime(10); //10 milli is 100hz
  yPID.SetSampleTime(10);
  zPID.SetSampleTime(10);
  xPID.SetOutputLimits(-MAX_SPEED, MAX_SPEED);
  yPID.SetOutputLimits(-MAX_SPEED, MAX_SPEED);
  zPID.SetOutputLimits(-MAX_SPEED, MAX_SPEED);
  xPID.SetMode(AUTOMATIC);
  yPID.SetMode(AUTOMATIC);
  zPID.SetMode(AUTOMATIC);
}

void setup(){
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(38400);
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);

  setupIMU();
  setupStepper();
  setupPID();
}

void loop(){
  timerstart = millis();
  readIMU();
  xPID.Compute();
  yPID.Compute();
  zPID.Compute();
  Serial.print("X-speed: "); Serial.print(OutputSpeedX);
  Serial.print("Y-speed: "); Serial.print(OutputSpeedY);
  Serial.print("Z-speed: "); Serial.print(OutputSpeedZ);
  speedCalculations();
  while(timer < sampleRate){
    runMotors(speed1, speed2, speed3);
    timer = millis()-timerstart;
  }
}
