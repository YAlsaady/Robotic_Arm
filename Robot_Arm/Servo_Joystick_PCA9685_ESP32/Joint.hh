#ifndef JOINT_H
#define JOINT_H
#include <stdint.h>
typedef uint8_t byte;

class joint {
protected:
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

  unsigned getPosition();
  unsigned getMax();
  unsigned getMin();
};

class double_joint : public joint {
private:
  uint8_t firstPin;
  uint8_t secendPin;

public:
  double_joint();
  void setJoint(uint8_t firstPin, uint8_t secendPin, unsigned minVal,
                unsigned maxVal, unsigned pos, unsigned startPos);
  void move();

  void moveMs(unsigned time);

  uint8_t moveDegree(int angle);
  
  void moveSteps(bool direction);
};

#endif // !JOINT_H

// vim:filetype=cpp
// vim:filetype=arduino
