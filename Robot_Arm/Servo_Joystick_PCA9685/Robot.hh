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

  float xPos = 0;
  float yPos = 350;
  float zPos = 150;
  int8_t grippingAngle = -90;
  int8_t rotionDegree = 90;

  uint8_t x1Pin;
  uint8_t y1Pin;
  uint8_t button1Pin;
  uint8_t x2Pin;
  uint8_t y2Pin;
  uint8_t button2Pin;

  uint16_t baseHight;
  uint16_t shoulderToElbow;
  uint16_t elbowToWrist;
  uint16_t gripperLength;

public:
  Robot();

  /* --- set --- */
  void setRobot(joint *base, joint *shoulder, joint *elbow, joint *wrist,
                joint *wristRot, joint *gripper);

  void setDimension(unsigned baseHight, unsigned shoulderToElbow,
                    unsigned elbowToWrist, unsigned gripperLength);

  void setJoystick(uint8_t x1Pin, uint8_t y1Pin, uint8_t button1Pin,
                   uint8_t x2Pin, uint8_t y2Pin, uint8_t button2Pin);

  /* --- move --- */
  uint8_t moveEndEffector(float xVal, float yVal, float zVal,
                          int8_t grippingAngle);

  void moveJoints(uint8_t baseAngle, uint8_t shoulderAngle, uint8_t elbowAngle,
                  uint8_t wristAngle, uint8_t gripperAngle,
                  uint8_t gripperOpening);

  void moveWithJoystick();

  uint8_t moveEndEffector_Joystick();

  /* --- Demos --- */
  uint8_t moveEndEffector_Demo();
};

#endif // !ROBOT_H

// vim:filetype=cpp
// vim:filetype=arduino
