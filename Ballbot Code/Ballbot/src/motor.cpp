#include "Arduino.h"
#include "Wire.h"

class Sweeper
{
  int i;
  int stepPin[];              // current servo position
  int dirPin[];        // increment to move for each interval
  int half_period;      // interval between updates

public:
  Motor(int step, int dir)
  {
    pinMode(stepPin,OUTPUT);
    pinMode(dirPin,OUTPUT);
    stepPin[];
  }

  void Update()
  {
    if((millis() - lastUpdate) > updateInterval)  // time to update
    {
      lastUpdate = millis();
      pos += increment;
      servo.write(pos);
      Serial.println(pos);
      if ((pos >= 180) || (pos <= 0)) // end of sweep
      {
        // reverse direction
        increment = -increment;
      }
    }
  }
};
