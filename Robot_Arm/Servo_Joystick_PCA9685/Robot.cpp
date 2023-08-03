#include <Arduino.h>
#include "Robot.hh"
#include "Joint.hh"

Robot::Robot(joint *base, joint *shoulder, joint *elbow, joint *wrist, joint *wristRot, joint *gripper) {
  this->base = base;
  this->shoulder = shoulder;
  this->elbow = elbow;
  this->wrist = wrist;
  this->wristRot = wristRot;
}

void Robot::moveWithJoystick(byte x1Pin, byte y1Pin, byte button1Pin, byte x2Pin, byte y2Pin, byte button2Pin) {
  if (analogRead(y1Pin) > 800) {
    this->base->moveSteps(HIGH);
  }
  if (analogRead(y1Pin) < 400) {
    this->base->moveSteps(LOW);
  }


  if (analogRead(x1Pin) > 800) {
    this->shoulder->moveSteps(HIGH);
  }
  if (analogRead(x1Pin) < 400) {
    this->shoulder->moveSteps(LOW);
  }

  if (analogRead(x2Pin) > 800) {
    this->elbow->moveSteps(HIGH);
  }
  if (analogRead(x2Pin) < 400) {
    this->elbow->moveSteps(LOW);
  }


  if (analogRead(y2Pin) > 800) {
    this->wrist->moveSteps(HIGH);
  }
  if (analogRead(y2Pin) < 400) {
    this->wrist->moveSteps(LOW);
  }


  if (digitalRead(button1Pin) == HIGH && digitalRead(button2Pin) == LOW) {
    this->wristRot->moveSteps(HIGH);
  }
  if (digitalRead(button1Pin) == LOW && digitalRead(button2Pin) == HIGH) {
    this->wristRot->moveSteps(LOW);
  }
}
// vim:filetype=cpp
// vim:filetype=arduino
