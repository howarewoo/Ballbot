#include <AccelStepper.h>

//AccelStepper Xaxis(1, 2, 5); // pin 2 = step, pin 5 = direction
//AccelStepper Yaxis(1, 3, 6); // pin 3 = step, pin 6 = direction
//AccelStepper Zaxis(1, 4, 7); // pin 4 = step, pin 7 = direction

AccelStepper Xaxis(1, 8, 7); // pin 8 = step, pin 7 = direction
AccelStepper Yaxis(1, 10, 9); // pin 10 = step, pin 9 = direction
AccelStepper Zaxis(1, 12, 11); // pin 12 = step, pin 11 = direction

void setup() {
  Xaxis.setMaxSpeed(400);
  Yaxis.setMaxSpeed(400);
  Zaxis.setMaxSpeed(400);
  Xaxis.setSpeed(45);
  Yaxis.setSpeed(25);
  Zaxis.setSpeed(80);
}

void loop() {  
   Xaxis.runSpeed();
   Yaxis.runSpeed();
   Zaxis.runSpeed();
   Serial.println("test");

}
