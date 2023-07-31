#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Joint.hh"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


Joint::Joint(byte pinNum, unsigned minVal, unsigned maxVal, unsigned pos, unsigned startPos) {
  this->pinNum = pinNum;
  this->maxVal = maxVal;
  this->minVal = minVal;
  this->pos = pos;
  this->steps = startPos;
}

void Joint::move() {
  pwm.writeMicroseconds(this->pinNum, this->pos);
}

void Joint::moveMs(unsigned time) {
  this->pos = time;
  pwm.writeMicroseconds(this->pinNum, this->pos);
}

void Joint::moveDegree(int angle) {
  this->pos = map(angle, maxVal, minVal, 0, 180);
  moveMs(this->pos);
}

void Joint::moveSteps(bool direction) {
  if (direction == HIGH && (this->pos < this->maxVal)) {
    this->pos += this->steps;
  } else if (direction == LOW && (this->pos > this->minVal)) {
    this->pos -= this->steps;
  }
  move();
}

unsigned Joint::position() {
  return pos;
}
