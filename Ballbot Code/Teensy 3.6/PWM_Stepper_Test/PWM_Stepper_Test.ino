#include <FrequencyTimer2.h>
#include <TimerOne.h>
#include <TimerThree.h>

#define DIR1 24
#define STEP1 2
#define DIR2 25
#define STEP2 3
#define DIR3 26
#define STEP3 5

long speed1, speed2, speed3;
long low = 1000;
long high = 2147483647;  // maximum value of long

void setup() {
  analogWrite(STEP1, 3);
  analogWrite(STEP2, 3);
  analogWrite(STEP3, 3);
  digitalWrite(DIR1,HIGH);
  analogWriteFrequency(STEP1, 1000); // pins 7, 8, 14, 35, 36, 37, 38 also change

  digitalWrite(DIR2,HIGH);
  analogWriteFrequency(STEP2, 1000); // pin 3 also changes

  digitalWrite(DIR3,LOW);
  analogWriteFrequency(STEP3, 1000); // pins 6, 9, 10, 20, 21, 22, 23 also change
}

void loop() {
//  delay(1000);
//  speed1 = random(low, high);
//  speed2 = random(low, high);
//  speed3 = random(low, high);
//
//  analogWriteFrequency(STEP1, speed1); // pins 7, 8, 14, 35, 36, 37, 38 also change
//  analogWriteFrequency(STEP2, speed2); // pin 3 also changes
//  analogWriteFrequency(STEP3, speed3); // pins 6, 9, 10, 20, 21, 22, 23 also
}
