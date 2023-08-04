#ifndef ROBOT_H
#define ROBOT_H
#include "Joint.hh"
#include <Arduino.h>
#include <stdint.h>

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

  uint8_t x1Pin;
  uint8_t y1Pin;
  uint8_t button1Pin;
  uint8_t x2Pin;
  uint8_t y2Pin;
  uint8_t button2Pin;

public:
  Robot();

  void setRobot(joint *base, joint *shoulder, joint *elbow, joint *wrist, joint *wristRot, joint *gripper);
  void setJoystick(byte x1Pin, byte y1Pin, byte button1Pin, byte x2Pin, byte y2Pin, byte button2Pin);

  //moveEndEfffector(xVal, yVal, zVal, RotionDegree);
  void moveJoints(byte baseAngle, byte shoulderAngle, byte elbowAngle, byte wristAngle, byte gripperAngle, byte gripperOpening);
  void moveWithJoystick();
};

#endif  // !ROBOT_H

// vim:filetype=cpp
// vim:filetype=arduino
