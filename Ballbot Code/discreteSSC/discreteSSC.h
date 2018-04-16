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
class SSC
{
  // user-accessible "public" interface
  public:
    SSC(double, double, double, double, double, double, double);

    void setMatrix(void);

    void update(double, int);

  // library-accessible "private" interface
  private:
    void doSomethingSecret(void);
};

#endif
