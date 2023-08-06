#include "Robot.hh"
#include "HardwareSerial.h"
#include "Joint.hh"
#include <Arduino.h>
#include <errno.h>
#include <math.h>

Robot::Robot() {}

void Robot::setRobot(joint *base, joint *shoulder, joint *elbow, joint *wrist,
                     joint *wristRot, joint *gripper) {
  this->base = base;
  this->shoulder = shoulder;
  this->elbow = elbow;
  this->wrist = wrist;
  this->wristRot = wristRot;
}

void Robot::setDimension(unsigned baseHight, unsigned shoulderToElbow,
                         unsigned elbowToWrist, unsigned gripperLength) {
  this->baseHight = baseHight;
  this->shoulderToElbow = shoulderToElbow;
  this->elbowToWrist = elbowToWrist;
  this->gripperLength = gripperLength;
  zPos = baseHight + gripperLength;
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

uint8_t Robot::moveEndEffector(float xVal, float yVal, float zVal,
                               int8_t gripperAngle, int8_t rotionDegree) {
                                 
  errno = 0;
  Serial.print("\t");
  Serial.print(xVal);
  Serial.print("\t");
  Serial.print(yVal);
  Serial.print("\t");
  Serial.print(zVal);
  Serial.print("\t");



  static uint32_t shoulderToElbow_square = shoulderToElbow * shoulderToElbow;
  static uint32_t elbowToWrist_square = elbowToWrist * elbowToWrist;

  float baseAngle, shoulderAngle, elbowAngle, wristAngle;
  float shoulderToWrist, shoulderToWrist_square;

  baseAngle = atan2(yVal, xVal);
  baseAngle = degrees(baseAngle);

  zVal = zVal - baseHight + (gripperLength * sin(radians(gripperAngle)));
  // yVal = yVal * yVal;
  // yVal += xVal * xVal;
  // yVal = sqrt(yVal);
  // yVal -= (gripperLength * cos(radians(gripperAngle)));
  yVal = sqrt((xVal * xVal) + (yVal * yVal)) -
         (gripperLength * cos(radians(gripperAngle)));
  Serial.print(yVal);
  Serial.print("\t");
  Serial.print(zVal);
  Serial.print("\t");
  shoulderToWrist_square = (zVal * zVal) + (yVal * yVal);
  shoulderToWrist = sqrtf(shoulderToWrist_square);
  shoulderAngle = atan2(zVal, yVal);
  shoulderAngle += acos((float(elbowToWrist_square) + shoulderToWrist_square -
                              float(shoulderToElbow_square)) /
                        float(2.0 * float(elbowToWrist) * shoulderToWrist));
  shoulderAngle = degrees(shoulderAngle);

  elbowAngle = acos((float(elbowToWrist_square) + float(shoulderToElbow_square) -
                          shoulderToWrist_square) /
                    (2.0 * float(elbowToWrist) * float(shoulderToElbow)));
  // Serial.print((shoulderToWrist));
  // Serial.print("\t");
  Serial.print((baseAngle));
  Serial.print("\t");
  Serial.print(shoulderAngle);
  Serial.print("\t");

  elbowAngle = degrees(elbowAngle);
  Serial.print(elbowAngle);
  Serial.print("\t");

  wristAngle = shoulderAngle + elbowAngle - gripperAngle;
  wristAngle = 0;
  // Serial.print(wristAngle);
  Serial.println("\t");

  // Serial.print(baseAngle);
  // Serial.print("\t");
  // Serial.print(shoulderAngle);
  // Serial.print("\t");
  // Serial.print(elbowAngle);
  // Serial.print("\t");
  // Serial.print(wristAngle);
  // Serial.print("\t");
  // Serial.print(rotionDegree);
  // Serial.println();
  errno = base->moveDegree(baseAngle);
  if (errno != 0) {
    // Serial.println(errno);
    return errno;
  }
  errno = shoulder->moveDegree(shoulderAngle);
  if (errno != 0) {
    Serial.println(errno);
    return errno;
  }
  errno = elbow->moveDegree(elbowAngle);
  if (errno != 0) {
    Serial.println(errno);
    return errno;
  }
  errno = wrist->moveDegree(wristAngle);
  if (errno != 0) {
    Serial.println(errno);
    return errno;
  }
  errno = wristRot->moveDegree(rotionDegree);
  if (errno != 0) {
    Serial.println(errno);
    return errno;
  }
  //Serial.print("\t");
  //Serial.print(errno);return 100;
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

uint8_t Robot::moveEndEffector_Joystick() {
  if (analogRead(x1Pin) > 800) {
    xPos++;
  }
  if (analogRead(x1Pin) < 400) {
    xPos--;
  }

  if (analogRead(y1Pin) > 800) {
    yPos++;
  }
  if (analogRead(y1Pin) < 400) {
    yPos--;
  }

  if (analogRead(x2Pin) > 800) {
    zPos++;
  }
  if (analogRead(x2Pin) < 400) {
    zPos--;
  }

  if (analogRead(y2Pin) > 800) {
    rotionDegree++;
  }
  if (analogRead(y2Pin) < 400) {
    rotionDegree--;
  }

  if (digitalRead(button1Pin) == HIGH && digitalRead(button2Pin) == LOW) {
    gripperAngle++;
  }
  if (digitalRead(button1Pin) == LOW && digitalRead(button2Pin) == HIGH) {
    gripperAngle--;
  }
  (moveEndEffector(xPos, yPos, zPos, gripperAngle, rotionDegree));

  return 0;
}

// vim:filetype=cpp
// vim:filetype=arduino
