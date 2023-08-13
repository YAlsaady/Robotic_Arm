#ifndef JOINT_H
#define JOINT_H
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

class joint {
private:
  byte pinNum;
  unsigned maxVal;
  unsigned minVal;
  unsigned pos;
  uint8_t angle;
  unsigned steps;

public:
  joint();

  void setJoint(byte pinNum, unsigned minVal, unsigned maxVal, unsigned pos,
                unsigned startPos);

  void move();

  void moveMs(unsigned time);

  uint8_t moveDegree(int angle);

  void moveSteps(bool direction);

  unsigned position();
};
#endif // !JOINT_H

// vim:filetype=cpp
// vim:filetype=arduino
