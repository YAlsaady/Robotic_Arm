#ifndef Joint_H
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class Joint {
private:
  byte pinNum;
  unsigned maxVal;
  unsigned minVal;
  unsigned pos;
  unsigned steps;

public:
  Joint();
  Joint(byte pinNum, unsigned minVal, unsigned maxVal, unsigned pos, unsigned startPos);

  void move();

  void moveMs(unsigned time);

  void moveDegree(int angle);

  void moveSteps(bool direction);

  unsigned position();
};
#endif  // !Joint_H
