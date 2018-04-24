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

// ensure this library description is only included once
#ifndef discreteSSC
#define discreteSSC
#define LIBRARY_VERSION	0.1.0


// library interface description
class SSC{
  // library-accessible "private" interface
  private:
    void doSomethingSecret(void);

    double myA[4][4];
    double myB[4][1];
    double myK[4];
    double myTs;
    long double mySetpoints[4][1];
    long double myX[4][1];
    long double myU[4][1];

    double sampleTime;
    int lastTime;
    int now;
    int i,j;

    long double myMat[4][4];
    long double myBK[4][4];

  // user-accessible "public" interface
  public:
    SSC(double [4][4], double [4][1], double [4], double, long double [4][1], long double [4][1], long double [4][1]);

    void setSSC(void);

    long double update(long double, int, int);
};

#endif
