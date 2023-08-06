#include "Joint.hh"
#include "Robot.hh"
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>
#include <errno.h>

#define TIME 10
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

  base.setJoint    (     0,      600,        2400,       1550,       STEPS       );
  shoulder.setJoint(     1,      750,        2000,       1360,       STEPS / 5   );
  elbow.setJoint   (     2,      630,        2050,       1340,       STEPS / 2   );
  wrist.setJoint   (     4,      570,        2340,       1455,       STEPS       );
  wristRot.setJoint(     3,      450,        2220,       1350,       STEPS       );
  gripper .setJoint(     5,      630,        2150,       1300,       STEPS       );

  myrobot.setRobot(&base, &shoulder, &elbow, &wrist, &wristRot, &gripper);
  myrobot.setJoystick(A0, A1, 2, A2, A3, 8);
  myrobot.setDimension(139, 226, 226, 60);

  pinMode(7, OUTPUT); // Relay
  digitalWrite(7, LOW);

  // myrobot.moveJoints(90, 90, 90, 90, 90, 0);
  delay(200);
}

void loop() {
  //myrobot.moveWithJoystick();
  myrobot.moveEndEffector_Joystick();
  //Serial.print(errno);
  // delay(TIME);
}

// vim:filetype=cpp
// vim:filetype=arduino
