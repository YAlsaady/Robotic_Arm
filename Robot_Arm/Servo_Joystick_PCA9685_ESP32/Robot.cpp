#include "Robot.hh"
#include "HardwareSerial.h"
#include "Joint.hh"
#include <Arduino.h>
#include <errno.h>
#include <math.h>

#define READHIGH 1000
#define READLOW 200
#define STEPS 2
#define TIME 02

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
  // zPos = baseHight + gripperLength;
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
                               int8_t grippingAngle) {
  static float shoulderToElbow_square = (shoulderToElbow * shoulderToElbow);
  static float elbowToWrist_square = (elbowToWrist * elbowToWrist);

  float baseAngle, shoulderAngle, elbowAngle, wristAngle;
  float shoulderToWrist, shoulderToWrist_square;

  baseAngle = degrees(atan2(yVal, xVal));

  Serial.print(xVal);
  Serial.print("\t");
  Serial.print(yVal);
  Serial.print("\t");
  Serial.print(zVal);
  Serial.print("\t");
  Serial.print(grippingAngle);
  Serial.print("\t");
  zVal = zVal - baseHight - (gripperLength * sin(radians(grippingAngle)));
  yVal = sqrt((xVal * xVal) + (yVal * yVal)) -
         (gripperLength * cos(radians(grippingAngle)));

  shoulderToWrist_square = (zVal * zVal) + (yVal * yVal);
  shoulderToWrist = sqrtf(shoulderToWrist_square);

  shoulderAngle = atan2(zVal, yVal);
  shoulderAngle += (acos((shoulderToElbow_square + (shoulderToWrist_square -
                         elbowToWrist_square)) /
                        (2.0 * float(shoulderToElbow) * shoulderToWrist)));
  if ( shoulderAngle == NAN) {return 2;}
  shoulderAngle = degrees(shoulderAngle);
  if ( shoulderAngle == NAN) {return 2;}

  elbowAngle = acos((float(elbowToWrist_square) +
                     float(shoulderToElbow_square) - shoulderToWrist_square) /
                    (2.0 * float(elbowToWrist) * float(shoulderToElbow)));
  elbowAngle = degrees(elbowAngle);

  wristAngle = 270 + grippingAngle - (shoulderAngle + elbowAngle) ;

  elbowAngle = elbowAngle - 90;
  if (baseAngle < 0 || baseAngle > 180) { return 1;}
  if (shoulderAngle < 0 || shoulderAngle > 90) { return 2;}
  if (elbowAngle < -20 || elbowAngle > 180) { return 3;}
  if (wristAngle < 0 || wristAngle > 180) { return 4;}
  // if (baseAngle < 0 || baseAngle > 180) { return 1;}
  // if (baseAngle < 0 || baseAngle > 180) { return 1;}
  base->moveDegree(baseAngle);
  shoulder->moveDegree(shoulderAngle);
  elbow->moveDegree(elbowAngle); 
  wrist->moveDegree(wristAngle);
  Serial.println();
  return 0;
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
  if (analogRead(y1Pin) > READHIGH) {
    this->base->moveSteps(HIGH);
  }
  if (analogRead(y1Pin) < READLOW) {
    this->base->moveSteps(LOW);
  }

  if (analogRead(x1Pin) > READHIGH) {
    this->shoulder->moveSteps(HIGH);
  }
  if (analogRead(x1Pin) < READLOW) {
    this->shoulder->moveSteps(LOW);
  }

  if (analogRead(x2Pin) > READHIGH) {
    this->elbow->moveSteps(HIGH);
  }
  if (analogRead(x2Pin) < READLOW) {
    this->elbow->moveSteps(LOW);
  }

  if (analogRead(y2Pin) > READHIGH) {
    this->wrist->moveSteps(HIGH);
  }
  if (analogRead(y2Pin) < READLOW) {
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
  if (analogRead(x1Pin) > READHIGH) {
    zPos += STEPS;
  }
  if (analogRead(x1Pin) < READLOW) {
    zPos -= STEPS;
  }

  if (analogRead(y1Pin) > READHIGH) {
    grippingAngle += STEPS;
  }
  if (analogRead(y1Pin) < READLOW) {
    grippingAngle -= STEPS;
  }

  if (analogRead(x2Pin) > READHIGH) {
    yPos -= STEPS;
  }
  if (analogRead(x2Pin) < READLOW) {
    yPos += STEPS;
  }

  if (analogRead(y2Pin) > READHIGH) {
    xPos -= STEPS;
  }
  if (analogRead(y2Pin) < READLOW) {
    xPos += STEPS;
  }

  if (digitalRead(button1Pin) == HIGH && digitalRead(button2Pin) == LOW) {
    rotionDegree += STEPS;
  }
  if (digitalRead(button1Pin) == LOW && digitalRead(button2Pin) == HIGH) {
    rotionDegree -= STEPS;
  }
  (moveEndEffector(xPos, yPos, zPos, grippingAngle));

  return 0;
}

uint8_t Robot::moveEndEffector_Demo() {
  for (uint16_t i = 265; i < 455; i+=1){
    moveEndEffector(0, i, 100, -85);
    delay(TIME);
  }
  delay(100*TIME);
  for (uint16_t i = 455; i > 265; i-=1){
    moveEndEffector(0, i, 100, -85);
    delay(TIME);
  }
  delay(1000*TIME);

  for (uint16_t i = 265; i < 350; i+=1){
    moveEndEffector(0, i, 100, -85);
    delay(TIME);
  }
  // myrobot.moveEndEffector(0, 350, 100, -85);
  for (int16_t i = 0; i > -270; i-=1){
    moveEndEffector(i, 350, 100, -85);
    delay(TIME);
  }
  delay(1000*TIME);

  for (int16_t i = -270; i < 270; i+=1){
    moveEndEffector(i, 350, 100, -85);
    delay(TIME);
  }
  delay(100*TIME);
  for (int16_t i = 270; i > 0; i-=1){
    moveEndEffector(i, 350, 100, -85);
    delay(TIME);
  }
  delay(1000*TIME);

  moveEndEffector(0, 350, 130, -85);
  delay(100*TIME);
  moveEndEffector(0, 350, 130, 10);
  delay(1000*TIME);

  for (uint16_t i = 100; i < 480; i+=1){
    moveEndEffector(0, 350, i, 10);
    delay(TIME);
  }
  delay(100*TIME);
  for (uint16_t i = 480; i > 100; i-=1){
    moveEndEffector(0, 350, i, 10);
    delay(TIME);
  }
}
// vim:filetype=cpp
// vim:filetype=arduino