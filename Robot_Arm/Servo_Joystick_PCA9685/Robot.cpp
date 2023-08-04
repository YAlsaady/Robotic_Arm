#include "Robot.hh"
#include "Joint.hh"
#include <Arduino.h>

Robot::Robot() {}

void Robot::setRobot(joint *base, joint *shoulder, joint *elbow, joint *wrist,
                     joint *wristRot, joint *gripper) {
  this->base = base;
  this->shoulder = shoulder;
  this->elbow = elbow;
  this->wrist = wrist;
  this->wristRot = wristRot;
}

void Robot::setJoystick(byte x1Pin, byte y1Pin, byte button1Pin, byte x2Pin,
                        byte y2Pin, byte button2Pin) {
  this->x1Pin = x1Pin;
  this->y1Pin = y1Pin;
  this->x2Pin = x2Pin;
  this->y2Pin = y2Pin;
  this->button1Pin = button1Pin;
  this->button2Pin = button2Pin;

  pinMode(x1Pin, INPUT); // Joystick 1 X
  pinMode(y1Pin, INPUT); // Joystick 1 Y
  pinMode(x2Pin, INPUT); // Joystick 2 X
  pinMode(y2Pin, INPUT); // Joystick 2 Y

  pinMode(button1Pin, INPUT);
  pinMode(button2Pin, INPUT);

  digitalWrite(button1Pin, HIGH);
  digitalWrite(button2Pin, HIGH);
}

void Robot::moveJoints(byte baseAngle, byte shoulderAngle, byte elbowAngle,
                       byte wristAngle, byte gripperAngle,
                       byte gripperOpening) {
  gripperOpening = map(gripperOpening, 0, 100, 0, 180);

  this->base->moveDegree(baseAngle);
  this->shoulder->moveDegree(shoulderAngle);
  this->elbow->moveDegree(elbowAngle);
  this->wrist->moveDegree(wristAngle);
  this->wristRot->moveDegree(gripperAngle);
  this->gripper->moveDegree(gripperOpening);
}

void Robot::moveWithJoystick() {
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
