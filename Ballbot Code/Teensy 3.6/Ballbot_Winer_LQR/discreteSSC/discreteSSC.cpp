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

SSC::SSC(double A[4][4], double B[4][1], double K[1][4], double Ts, double Setpoints[4][1], double U[4][1], double X[4][1])
{
  // initialize this instance's variables
  myA = A;
  myB = B;
  myK = K;
  myTs = Ts;
  mySetpoints = Setpoints;
  myX = X;
  myU = U;

  // do whatever is required to initialize the library
  SSC::setMatrix();

  sampleTime = myTs * 1000000;
  lastTime = micros()-SampleTime;
}

// Public Methods //////////////////////////////////////////////////////////////
// Functions available in Wiring sketches, this library, and other libraries

void SSC::setMatrix(){
  int i, j;

  // Calculate B*K
  for (i = 0; i < 4; i++){
    for (j = 0; j < 4; j++){
      myBK[i][j] += myB[i][0]*myK[0][j];
    }
  }

  // Calculate A-(B*K)
  for (i = 0; i < 4; i++){
    for (j = 0; j < 4; j++){
      myMat[i][j] = myA[i][j]-myBK[i][j];
    }
  }
}

double SSC::update(double value, int inState, int outState){
  // eventhough this function is public, it can access
  // and modify this library's private variables
  unsigned long now = micros();
  unsigned long timeChange = (now - lastTime);
  if(timeChange>=SampleTime)
  {
    myU[inState][0] = value;
    int i, j;
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

    myU = myX;
    lastTime = now;
    return myX[outState][0];
  }
}

// Private Methods /////////////////////////////////////////////////////////////
// Functions only available to other functions in this library

void SSC::doSomethingSecret(void)
{
}
