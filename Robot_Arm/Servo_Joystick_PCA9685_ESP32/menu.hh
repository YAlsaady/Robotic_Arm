#include "Joint.hh"
#include "Robot.hh"
#include "WString.h"
#include <Adafruit_PWMServoDriver.h>
// #include <Arduino.h>
#include <ESP32Encoder.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <errno.h>
#include <stdint.h>

void start();
void IRAM_ATTR isr();
void test();
