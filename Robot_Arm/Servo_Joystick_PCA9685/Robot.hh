#ifndef ROBOT_H
#include "Joint.hh"
#include <Arduino.h>

class Robot {
private:
  joint base;
  joint shoulder;
  joint elbow;
  joint wrist;
  joint wristRot;
  joint gripper;
  int xPos;
  int yPos;
  int zPos;

public:
  Robot(joint base, joint shoulder, joint elbow, joint wrist, joint wristRot,
        joint gripper);
};

#endif // !ROBOT_H
