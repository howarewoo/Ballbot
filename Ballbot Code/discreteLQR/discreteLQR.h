/*
  Test.h - Test library for Wiring - description
  Copyright (c) 2006 John Doe.  All right reserved.
*/

// ensure this library description is only included once
#ifndef discreteLQR
#define discreteLQR
#define LIBRARY_VERSION	0.1.0


// library interface description
class LQR
{
  // user-accessible "public" interface
  public:
    LQR(double, double, double, double, double, double, double);

    void setMatrix(void);

    void update(double, int);

  // library-accessible "private" interface
  private:
    void doSomethingSecret(void);
};

#endif
