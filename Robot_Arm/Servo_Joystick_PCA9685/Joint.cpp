#include "Joint.hh"
#include "HardwareSerial.h"

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

extern Adafruit_PWMServoDriver pwm;
extern HardwareSerial Serial;

joint::joint() {}

void joint::setJoint(uint8_t pinNum, uint16_t minVal, uint16_t maxVal,
                     uint16_t pos, uint16_t startPos) {
  this->pinNum = pinNum;
  this->maxVal = maxVal;
  this->minVal = minVal;
  this->pos = pos;
  this->steps = startPos;
}

void joint::move() { pwm.writeMicroseconds(this->pinNum, this->pos); }

void joint::moveMs(uint16_t time) {
  this->pos = time;
  pwm.writeMicroseconds(this->pinNum, this->pos);
}

uint8_t joint::moveDegree(uint8_t angle) {
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
  pwm.writeMicroseconds(this->pinNum, this->pos);
}

unsigned joint::position() { return pos; }

// vim:filetype=cpp
// vim:filetype=arduino
