#ifndef ROBOT_H
#define ROBOT_H
#include "Joint.hh"
#include <Arduino.h>

class Robot {
private:
  joint *base;
  joint *shoulder;
  joint *elbow;
  joint *wrist;
  joint *wristRot;
  joint *gripper;
  int xPos;
  int yPos;
  int zPos;

public:
  Robot();

  void setRobot(joint *base, joint *shoulder, joint *elbow, joint *wrist, joint *wristRot, joint *gripper);

  //moveEndEfffector(xVal, yVal, zVal, RotionDegree);
  //moveJoints();
  void moveWithJoystick(byte x1Pin, byte y1Pin, byte button1Pin, byte x2Pin, byte y2Pin, byte button2Pin);
};

#endif  // !ROBOT_H
// vim:filetype=cpp
// vim:filetype=arduino
