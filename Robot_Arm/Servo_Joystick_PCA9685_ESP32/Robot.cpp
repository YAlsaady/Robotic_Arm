#include "Robot.hh"
#include "Joint.hh"
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <errno.h>
#include <math.h>

extern LiquidCrystal_I2C lcd;

#define STEPS 1
#define TIME 05

Robot::Robot() {}

/* --- set --- */
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
                        byte y2Pin, byte button2Pin, uint16_t readHigh,
                        uint16_t readLow) {
  this->x1Pin = x1Pin;
  this->y1Pin = y1Pin;
  this->x2Pin = x2Pin;
  this->y2Pin = y2Pin;
  this->button1Pin = button1Pin;
  this->button2Pin = button2Pin;
  this->readHigh = readHigh;
  this->readLow = readLow;

  pinMode(x1Pin, INPUT); // Joystick 1 X
  pinMode(y1Pin, INPUT); // Joystick 1 Y
  pinMode(x2Pin, INPUT); // Joystick 2 X
  pinMode(y2Pin, INPUT); // Joystick 2 Y

  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);

  digitalWrite(button1Pin, HIGH);
  digitalWrite(button2Pin, HIGH);
}

/* --- move --- */
uint8_t Robot::moveEndEffector(float xVal, float yVal, float zVal,
                               int8_t grippingAngle) {
  static float shoulderToElbow_square = (shoulderToElbow * shoulderToElbow);
  static float elbowToWrist_square = (elbowToWrist * elbowToWrist);

  float baseAngle, shoulderAngle, elbowAngle, wristAngle;
  float shoulderToWrist, shoulderToWrist_square;

  baseAngle = degrees(atan2(yVal, xVal));

  zVal = zVal - baseHight - (gripperLength * sin(radians(grippingAngle)));
  yVal = sqrt((xVal * xVal) + (yVal * yVal)) -
         (gripperLength * cos(radians(grippingAngle)));

  shoulderToWrist_square = (zVal * zVal) + (yVal * yVal);
  shoulderToWrist = sqrtf(shoulderToWrist_square);

  shoulderAngle = atan2(zVal, yVal);
  shoulderAngle += (acos((shoulderToElbow_square +
                          (shoulderToWrist_square - elbowToWrist_square)) /
                         (2.0 * float(shoulderToElbow) * shoulderToWrist)));
  if (shoulderAngle == NAN) {
    return 2;
  }
  shoulderAngle = degrees(shoulderAngle);
  if (shoulderAngle == NAN) {
    return 2;
  }

  elbowAngle = acos((float(elbowToWrist_square) +
                     float(shoulderToElbow_square) - shoulderToWrist_square) /
                    (2.0 * float(elbowToWrist) * float(shoulderToElbow)));
  elbowAngle = degrees(elbowAngle);

  elbowAngle = elbowAngle - 90;

  wristAngle = 180 + grippingAngle - (shoulderAngle + elbowAngle);

  base->moveDegree(baseAngle);
  shoulder->moveDegree(shoulderAngle);
  elbow->moveDegree(elbowAngle);
  wrist->moveDegree(wristAngle);
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
  if (analogRead(y1Pin) > readHigh) {
    this->base->moveSteps(HIGH);
  }
  if (analogRead(y1Pin) < readLow) {
    this->base->moveSteps(LOW);
  }

  if (analogRead(x1Pin) > readHigh) {
    this->shoulder->moveSteps(HIGH);
  }
  if (analogRead(x1Pin) < readLow) {
    this->shoulder->moveSteps(LOW);
  }

  if (analogRead(x2Pin) > readHigh) {
    this->elbow->moveSteps(HIGH);
  }
  if (analogRead(x2Pin) < readLow) {
    this->elbow->moveSteps(LOW);
  }

  if (analogRead(y2Pin) > readHigh) {
    this->wrist->moveSteps(HIGH);
  }
  if (analogRead(y2Pin) < readLow) {
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
  /* --- Move in Z direction --- */
  if (analogRead(x1Pin) > readHigh) {
    zPos += STEPS;
  }
  if (analogRead(x1Pin) < readLow) {
    zPos -= STEPS;
  }

  /* --- Change the Angle of the Gripper --- */
  if (analogRead(y1Pin) > readHigh) {
    grippingAngle += STEPS;
  }
  if (analogRead(y1Pin) < readLow) {
    grippingAngle -= STEPS;
  }

  /* --- Move in Y direction --- */
  if (analogRead(x2Pin) > readHigh) {
    yPos -= STEPS;
  }
  if (analogRead(x2Pin) < readLow) {
    yPos += STEPS;
  }

  /* --- Move in X direction --- */
  if (analogRead(y2Pin) > readHigh) {
    xPos -= STEPS;
  }
  if (analogRead(y2Pin) < readLow) {
    xPos += STEPS;
  }

  if (digitalRead(button1Pin) == HIGH && digitalRead(button2Pin) == LOW) {
    rotionDegree += STEPS;
  }
  if (digitalRead(button1Pin) == LOW && digitalRead(button2Pin) == HIGH) {
    rotionDegree -= STEPS;
  }
  wristRot->moveDegree(rotionDegree);
  moveEndEffector(xPos, yPos, zPos, grippingAngle);

  return 0;
}

/* --- Demos ---*/
uint8_t Robot::moveEndEffector_Demo() {
  /* --- Move in Y direction --- */
  for (uint16_t i = 265; i < 455; i += 1) {
    moveEndEffector(0, i, 120, -85);
    delay(TIME);
  }
  delay(100 * TIME);
  for (uint16_t i = 455; i > 265; i -= 1) {
    moveEndEffector(0, i, 120, -85);
    delay(TIME);
  }
  delay(100 * TIME);

  for (uint16_t i = 265; i < 350; i += 1) {
    moveEndEffector(0, i, 120, -85);
    delay(TIME);
  }

  /* --- Move in X direction --- */
  for (int16_t i = 0; i > -270; i -= 1) {
    moveEndEffector(i, 350, 120, -85);
    delay(TIME);
  }
  delay(100 * TIME);

  for (int16_t i = -270; i < 270; i += 1) {
    moveEndEffector(i, 350, 120, -85);
    delay(TIME);
  }
  delay(100 * TIME);
  for (int16_t i = 270; i > 0; i -= 1) {
    moveEndEffector(i, 350, 120, -85);
    delay(TIME);
  }
  delay(100 * TIME);

  moveEndEffector(0, 350, 130, -85);
  delay(100 * TIME);
  moveEndEffector(0, 350, 130, 10);
  delay(100 * TIME);

  /* --- Move in Z direction --- */
  for (uint16_t i = 100; i < 480; i += 1) {
    moveEndEffector(0, 350, i, 10);
    delay(TIME);
  }
  delay(100 * TIME);
  for (uint16_t i = 480; i > 100; i -= 1) {
    moveEndEffector(0, 350, i, 10);
    delay(TIME);
  }

  return 0;
}

void Robot::lcdPrint() {
  lcd.setCursor(0, 2);
  lcd.print("XPos=" + String(xPos) + "  ");
  lcd.setCursor(11, 2);
  lcd.print("YPos=" + String(yPos) + "  ");
  lcd.setCursor(0, 3);
  lcd.print("ZPos=" + String(zPos) + "  ");
}

// vim:filetype=cpp
// vim:filetype=arduino
