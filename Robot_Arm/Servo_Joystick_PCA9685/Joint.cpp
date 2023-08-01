#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Joint.hh"

extern Adafruit_PWMServoDriver pwm;
//joint::joint() {}

joint::joint(byte pinNum, unsigned minVal, unsigned maxVal, unsigned pos, unsigned startPos) {
  this->pinNum = pinNum;
  this->maxVal = maxVal;
  this->minVal = minVal;
  this->pos = pos;
  this->steps = startPos;
}

void joint::move() {
  pwm.writeMicroseconds(this->pinNum, this->pos);
}

void joint::moveMs(unsigned time) {
  this->pos = time;
  move();
}

void joint::moveDegree(int angle) {
  if (minVal < 1000) {
    this->pos = map(angle, maxVal, minVal, 0, 180);
  } else {
    this->pos = map(angle, maxVal, minVal, 0, 90);
  }
  move();
}

void joint::moveSteps(bool direction) {
  if (direction == HIGH && (this->pos < this->maxVal)) {
    this->pos += this->steps;
  } else if (direction == LOW && (this->pos > this->minVal)) {
    this->pos -= this->steps;
  }
  move();
}

unsigned joint::position() {
  return pos;
}
// vim:filetype=arduino
