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

  int xPos = 0;
  int yPos = 100;
  int zPos = 0;
  int8_t gripperAngle = -90; 
  int8_t rotionDegree = 90;

  uint8_t x1Pin;
  uint8_t y1Pin;
  uint8_t button1Pin;
  uint8_t x2Pin;
  uint8_t y2Pin;
  uint8_t button2Pin;

  unsigned baseHight;
  unsigned shoulderToElbow;
  unsigned elbowToWrist;
  unsigned gripperLength;

public:
  Robot();

  /* --- set --- */
  void setRobot(joint *base, joint *shoulder, joint *elbow, joint *wrist,
                joint *wristRot, joint *gripper);

  void setDimension(unsigned baseHight, unsigned shoulderToElbow,
                    unsigned elbowToWrist, unsigned gripperLength);

  void setJoystick(byte x1Pin, byte y1Pin, byte button1Pin, byte x2Pin,
                   byte y2Pin, byte button2Pin);

  /* --- move --- */
  uint8_t moveEndEffector(float xVal, float yVal, float zVal,
                          int8_t gripperAngle, int8_t rotionDegree);

  void moveJoints(byte baseAngle, byte shoulderAngle, byte elbowAngle,
                  byte wristAngle, byte gripperAngle, byte gripperOpening);

  void moveWithJoystick();
  uint8_t moveEndEffector_Joystick();
};

#endif // !ROBOT_H

// vim:filetype=cpp
// vim:filetype=arduino
