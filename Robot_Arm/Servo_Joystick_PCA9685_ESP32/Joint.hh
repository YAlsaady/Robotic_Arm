#ifndef JOINT_H
#define JOINT_H
#include <stdint.h>

class joint {
private:
  uint8_t pinNum;
  unsigned maxVal;
  unsigned minVal;
  unsigned pos;
  uint8_t angle;
  unsigned steps;

public:
  joint();

  void setJoint(uint8_t pinNum, unsigned minVal, unsigned maxVal, unsigned pos,
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
