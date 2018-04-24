/*
* discreteSSC.h - Library implementation of a State Space Controller
*
* by Adam Woo
*
* The goal of this library is to implement state space controls in C++ or
* Arduino without the need for external matrix or linear algrbra libraries.
*
* TODO:
* - Read number of states
* - Add full state estimation
*/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "discreteSSC.h"

// Constructor /////////////////////////////////////////////////////////////////
// Function that handles the creation and setup of instances

SSC::SSC(double A[4][4], double B[4][1], double K[4], double Ts, long double Setpoints[4][1], long double U[4][1], long double X[4][1]){
  // initialize this instance's variables
  for (int i = 0; i < 4; i++){
    for (int j = 0; j < 4; j++){
      myA[i][j] = A[i][j];
      myB[i][j] = B[i][j];
      myK[i] = K[i];
      myTs = Ts;
      mySetpoints[i][j] = Setpoints[i][j];
      myU[i][j] = U[i][j];
      myX[i][j] = X[i][j];
    }

    // do whatever is required to initialize the library
    sampleTime = myTs * 1000000;
    lastTime = micros()-sampleTime;
  }
}

// Public Methods //////////////////////////////////////////////////////////////
// Functions available in Wiring sketches, this library, and other libraries

void SSC::setSSC(){
  // Calculate B*K
  for (i = 0; i < 4; i++){
    for (j = 0; j < 4; j++){
      myBK[i][j] += myB[i][0]*myK[j];
    }
  }

  // Calculate A-(B*K)
  for (i = 0; i < 4; i++){
    for (j = 0; j < 4; j++){
      myMat[i][j] = myA[i][j]-myBK[i][j];
    }
  }
}

long double SSC::update(long double value, int inState, int outState){
  // eventhough this function is public, it can access
  // and modify this library's private variables
  unsigned long now = micros();
  unsigned long timeChange = (now - lastTime);
  if(timeChange>=sampleTime){
    Serial.println(timeChange);
    myU[inState][0] = value;
    double myError[4][1];

    // From matlab:
    // y(i,:)=(((A-B*K)*(y(i-1,:)'-[dest; 0; pi; 0]))*Ts)+y(i-1,:)';

    // (y(i-1,:)'-[dest; 0; pi; 0]) = Inputs - Setpoints = Error
    for (i = 0; i < 4; i++){
      myError[i][0] = myU[i][0]-mySetpoints[i][0];
    }

    // ((A-B*K)*(y(i-1,:)'-[dest; 0; pi; 0])) = myMat*Error = Y_dot
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        myX[i][0] += myMat[i][j] * myError[j][0];
      }
    }

    // (((A-B*K)*(y(i-1,:)'-[dest; 0; pi; 0]))*Ts) = Y_dot * Ts = Y_dot_dis
    for (int i = 0; i < 4; i++) {
      myX[i][0] *= myTs;
    }

    // (((A-B*K)*(y(i-1,:)'-[dest; 0; pi; 0]))*Ts)+y(i-1,:)' = X_dot_dis + Inputs
    for (int i = 0; i < 4; i++) {
      myX[i][0] += myU[i][0];
    }

    for (int i = 0; i < 4; i++) {
      myU[i][0] = myX[i][0];
    }

    lastTime = now;
    return myX[outState][0];
  }
}

// Private Methods /////////////////////////////////////////////////////////////
// Functions only available to other functions in this library

void SSC::doSomethingSecret(void)
{
}
