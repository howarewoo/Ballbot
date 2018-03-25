#include <AccelStepper.h>

AccelStepper Xaxis(1, 22, 23); // pin 8 = step, pin 7 = direction
//AccelStepper Yaxis(1, 4, 7); // pin 4 = step, pin 7 = direction
//AccelStepper Zaxis(1, 5, 9); // pin 5 = step, pin 8 = direction

void setup() {
//  Xaxis.setMaxSpeed(400);
//  Yaxis.setMaxSpeed(400);
//  Zaxis.setMaxSpeed(400);
  Xaxis.setSpeed(100);
//  Yaxis.setSpeed(25);
//  Zaxis.setSpeed(80);
}

void loop() {  
   Xaxis.runSpeed();
//   Yaxis.runSpeed();
//   Zaxis.runSpeed();
}
