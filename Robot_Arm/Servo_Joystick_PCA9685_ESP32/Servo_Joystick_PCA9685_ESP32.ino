// #include "/home/yaman/.arduino15/packages/esp32/hardware/esp32/2.0.12/cores/esp32/Arduino.h",
#include <Arduino.h>
#include "Joint.hh"
#include "Robot.hh"
#include "esp32-hal.h"
#include "menu.hh"
#include "WString.h"
#include <Adafruit_PWMServoDriver.h>
#include <ESP32Encoder.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <errno.h>
#include <stdint.h>
#include "config.h"

#define TIME 02

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

joint base;
joint shoulder1;
joint shoulder2;
joint elbow;
joint wrist;
joint wristRot;
joint gripper;

double_joint shoulder;

Robot myrobot;

//LiquidCrystal_I2C lcd(0x27, 20, 4);
//ESP32Encoder encoder;

void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(10);

  //lcd.init();
  //lcd.backlight();
  //lcd.setCursor(0, 0);
  //lcd.print("Robotic Arm Menu:");

  //encoder.attachHalfQuad(27, 26);
  //encoder.setCount(0);
  //start();

  base.setJoint       (BASE);
  elbow.setJoint      (ELBOW);
  wrist.setJoint      (WRIST);
  wristRot.setJoint   (WRISTROT);
  gripper.setJoint    (GRIPPER);

  shoulder.setJoint(SHOULDER);
  // shoulder2.setJoint(     6,      930,        1530,       1530,       STEPS / 5   );

  myrobot.setRobot(&base, &shoulder, &elbow, &wrist, &wristRot, &gripper);
  myrobot.setJoystick(JOYSTICK_PAR);
  myrobot.setDimension(DIMENSION);

  pinMode(25, OUTPUT); // Relay
  digitalWrite(25, LOW);

  myrobot.moveJoints(90,90, 90, 90, 90, 0);
  // myrobot.moveEndEffector(0, 300, 100, -90);
  delay(200);
  // base.moveDegree(0);
  // delay(200);
  // base.moveDegree(180);
}

void loop() {
  // shoulder.moveMs(1550);
  // shoulder.moveDegree(110);
  // shoulder.moveMs(shoulder.getMin());
  // delay(3000);
  // shoulder.moveMs(shoulder.getMax());
  // delay(3000);
  // myrobot.moveEndEffector_Joystick();
  myrobot.moveWithJoystick();
  //test();
  // int val = 180;
  // shoulder.moveDegree(val);
  // shoulder2.moveDegree(180 - val);
  /*
  int i;
  for (i = 0; i <= 180; i++) {
    shoulder.moveDegree(i);
    shoulder2.moveDegree(180 - i);
    delay(20);
  }
  */
  // delay(1000);
  // for (i = 180; i >= 0; i--) {
  //   shoulder.moveDegree(i);
  //   shoulder2.moveDegree(180 - i);
  //   delay(20);
  // }
  // delay(1000);
}

// vim:filetype=cpp
// vim:filetype=arduino
