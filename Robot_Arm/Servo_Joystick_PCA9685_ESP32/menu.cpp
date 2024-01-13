#include "Joint.hh"
#include "Robot.hh"
#include "menu.hh"
#include "WString.h"
// #include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <errno.h>
#include <stdint.h>

extern Robot myrobot;
extern LiquidCrystal_I2C lcd;
extern ESP32Encoder encoder;

bool isSelected = false;

void start() {
  attachInterrupt(digitalPinToInterrupt(14), isr, RISING);
}

void IRAM_ATTR isr() {
  static unsigned long lastUpdateTime = 0;
  if (millis() - lastUpdateTime < 200) {
    return;
  }
  isSelected = true;
  lastUpdateTime = millis();
}

void test() {
  static int val = 0;
  static unsigned long lastUpdateTime = 0;
  static String option;
  static int selected = 0;
  val = encoder.getCount() / 2;

  if (val > 2) {
    encoder.setCount(0);
  }
  if (val < 0) {
    encoder.setCount(4);
  }
  switch (val) {
  case 0:
    option = "Joystick: Gripper";
    break;
  case 1:
    option = "Joystick: Joints ";
    break;
  case 2:
    option = "Demo             ";
    break;
  default:
    break;
  }
  if (isSelected) {
    selected = val;
    isSelected = false;
  }
  switch (selected) {
  case 0:
    myrobot.moveEndEffector_Joystick();
    break;

  case 1:
    myrobot.moveWithJoystick();
    break;

  case 2:
    lcd.clear();
    lcd.setCursor(5, 2);
    lcd.print("DEMO X Y Z");
    myrobot.moveEndEffector_Demo();
    selected = 0;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Robotic Arm Menu:");
    break;

  default:
    break;
  }

  if (millis() - lastUpdateTime >= 500) {
    option = " > " + option;
    lcd.setCursor(0, 1);
    lcd.print(option);
    myrobot.lcdPrint();
    lastUpdateTime = millis();
  }
}
