#include "Joint.hh"
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

extern Adafruit_PWMServoDriver pwm;
extern HardwareSerial Serial;

joint::joint() {}

void joint::setJoint(byte pinNum, unsigned minVal, unsigned maxVal,
                     unsigned pos, unsigned startPos) {
  this->pinNum = pinNum;
  this->maxVal = maxVal;
  this->minVal = minVal;
  this->pos = pos;
  this->steps = startPos;
}

void joint::move() { pwm.writeMicroseconds(this->pinNum, this->pos); }

void joint::moveMs(unsigned time) {
  this->pos = time;
  pwm.writeMicroseconds(this->pinNum, this->pos);
}

uint8_t joint::moveDegree(int angle) {
  if (angle < 0)
    return 0;
  if (maxVal > 1000 && angle > 180)
    return 0;
  if (maxVal < 1000 && angle > 90)
    return 0;
  this->angle = angle;
  if (maxVal > 1800) {
    this->pos = map(angle, 0, 180, minVal, maxVal);
  } else {
    this->pos = map(angle, 0, 90, minVal, maxVal);
  }

  move();

  return 0;
}

void joint::moveSteps(bool direction) {
  if (direction == HIGH && (this->pos < this->maxVal)) {
    this->pos += this->steps;
  } else if (direction == LOW && (this->pos > this->minVal)) {
    this->pos -= this->steps;
  }

  move();
}

unsigned joint::getPosition() { return pos; }
unsigned joint::getMax() { return maxVal; }
unsigned joint::getMin() { return minVal; }

double_joint::double_joint() {}
void double_joint::setJoint(uint8_t firstPin, uint8_t secendPin,
                            unsigned minVal, unsigned maxVal, unsigned pos,
                            unsigned startPos) {
  this->firstPin = firstPin;
  this->secendPin = secendPin;
  this->maxVal = maxVal;
  this->minVal = minVal;
  this->pos = pos;
  this->steps = startPos;
}
void double_joint::move() {
  pwm.writeMicroseconds(this->firstPin, this->pos);
  pwm.writeMicroseconds(this->secendPin, maxVal+ minVal - this->pos);
}

void double_joint::moveMs(unsigned time) {
  this->pos = time;
  pwm.writeMicroseconds(this->firstPin, this->pos);
  // pwm.writeMicroseconds(this->secendPin, this->pos);
  pwm.writeMicroseconds(this->secendPin, maxVal+ minVal - this->pos);
}

uint8_t double_joint::moveDegree(int angle) {
  if (angle < 0)
    return 0;
  if (maxVal > 1000 && angle > 180)
    return 0;
  if (maxVal < 1000 && angle > 90)
    return 0;
  this->angle = angle;
  if (maxVal > 1800) {
    this->pos = map(angle, 0, 180, minVal, maxVal);
  } else {
    this->pos = map(angle, 0, 90, minVal, maxVal);
  }

  move();

  return 0;
}

void double_joint::moveSteps(bool direction) {
  if (direction == HIGH && (this->pos < this->maxVal)) {
    this->pos += this->steps;
  } else if (direction == LOW && (this->pos > this->minVal)) {
    this->pos -= this->steps;
  }

  move();
}
// vim:filetype=cpp
// vim:filetype=arduino
