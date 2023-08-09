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

  void setJoint(uint8_t pinNum, uint16_t minVal, uint16_t maxVal, uint16_t pos,
                uint16_t startPos);

  void move();

  void moveMs(uint16_t time);

  uint8_t moveDegree(uint8_t angle);

  void moveSteps(bool direction);

  unsigned position();
};

#endif // !JOINT_H

// vim:filetype=cpp
// vim:filetype=arduino
