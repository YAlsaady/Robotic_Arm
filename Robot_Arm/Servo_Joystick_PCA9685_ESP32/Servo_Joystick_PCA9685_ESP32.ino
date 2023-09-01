#include "Joint.hh"
#include "Robot.hh"
#include "WString.h"
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <errno.h>
#include <stdint.h>

#define TIME 02
#define STEPS 5

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

joint base;
joint shoulder;
joint elbow;
joint wrist;
joint wristRot;
joint gripper;

Robot myrobot;

void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(10);

  // shoulder.setJoint(     1,      950,        1550,       1550,       STEPS / 5   );
  base.setJoint    (     0,      600,        2400,       1550,       STEPS       );
  shoulder.setJoint(     1,      950,        2150,       1550,       STEPS / 5   );
  elbow.setJoint   (     2,      780,        1940,       1360,       STEPS / 2   );
  wrist.setJoint   (     4,      570,        2340,       1455,       STEPS       );
  wristRot.setJoint(     3,      490,        2220,       1400,       STEPS       );
  gripper.setJoint (     5,      600,         940,        940,       STEPS       );

  myrobot.setRobot(&base, &shoulder, &elbow, &wrist, &wristRot, &gripper);
  myrobot.setJoystick(35, 34, 32, 39, 36, 33, 4000, 800);
  myrobot.setDimension(139, 226, 226, 184);

  pinMode(25, OUTPUT); // Relay
  digitalWrite(25, LOW);


  // myrobot.moveJoints(90, 90, 90, 90, 90, 0);
  // myrobot.moveEndEffector(0, 300, 100, -90);
  delay(200);
}

void loop() {
  //myrobot.moveJoints(90, 90, 0, 0, 90, 0);
  myrobot.moveEndEffector_Joystick();
  //myrobot.moveEndEffector(0, 300, 200);
}

// vim:filetype=cpp
// vim:filetype=arduino
