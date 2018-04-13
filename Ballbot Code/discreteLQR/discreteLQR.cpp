/*
discreteLQR.h - Library implementation of a Linear Quadradic Regulator
*/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "discreteLQR.h"

// Constructor /////////////////////////////////////////////////////////////////
// Function that handles the creation and setup of instances

LQR::LQR(double A[4][1], double B[4][1], double K[1][4], double Ts, double Setpoints[4][1], double Inputs[4][1], double Outputs[4][1])
{
  // initialize this instance's variables
  myA = A;
  myB = B;
  myK = K;
  myTs = Ts;
  mySetpoints = Setpoints;
  myOutputs = Outputs;
  myInputs = Inputs;

  // do whatever is required to initialize the library
  LQR::setMatrix();

  sampleTime = myTs * 1000000;
  lastTime = micros()-SampleTime;
}

// Public Methods //////////////////////////////////////////////////////////////
// Functions available in Wiring sketches, this library, and other libraries

void LQR::setMatrix(){
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
      myMat[i][j] = A[i][j]-BK[i][j];
    }
  }
}

double LQR::update(double Inputs, int inState, int outState){
  // eventhough this function is public, it can access
  // and modify this library's private variables
  unsigned long now = micros();
  unsigned long timeChange = (now - lastTime);
  if(timeChange>=SampleTime)
  {
    myInputs[inState][0] = Input;
    int i, j;
    double myError[4][1];

    // From matlab:
    // y(i,:)=(((A-B*K)*(y(i-1,:)'-[dest; 0; pi; 0]))*Ts)+y(i-1,:)';

    // (y(i-1,:)'-[dest; 0; pi; 0]) = Inputs - Setpoints = Error
    for (i = 0; i < 4; i++){
      for (j = 0; j < 4; j++){
        myError[i][j] = myInputs[i][j]-mySetpoints[i][j];
      }
    }

    // ((A-B*K)*(y(i-1,:)'-[dest; 0; pi; 0])) = myMat*Error = Y_dot
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        myOutputs[i][0] += myMat[i][j] * myError[j][0];
      }
    }

    // (((A-B*K)*(y(i-1,:)'-[dest; 0; pi; 0]))*Ts) = Y_dot * Ts = Y_dot_dis
    for (int i = 0; i < 4; i++) {
      myOutputs[i][0] *= myTs;
    }

    // (((A-B*K)*(y(i-1,:)'-[dest; 0; pi; 0]))*Ts)+y(i-1,:)' = Y_dot_dis + Inputs
    for (int i = 0; i < 4; i++) {
      myOutputs[i][0] += myInputs[i][0];
    }

    myInputs = myOutputs;
    lastTime = now;
    return myOutputs[outState][0];
  }
}

// Private Methods /////////////////////////////////////////////////////////////
// Functions only available to other functions in this library

void LQR::doSomethingSecret(void)
{
}
