#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Joint.hh"
#include "Robot.hh"

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
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

  base.setJoint    (     0,      600,        2400,       1550,       STEPS       );
  shoulder.setJoint(     1,      750,        2000,       1360,       STEPS / 2   );
  elbow.setJoint   (     2,      630,        2050,       1340,       STEPS       );
  wrist.setJoint   (     4,      570,        2340,       1455,       STEPS / 2   );
  wristRot.setJoint(     3,      450,        2220,       1350,       STEPS       );
  gripper .setJoint(     5,      630,        2150,       1300,       STEPS       );

  myrobot.setRobot (&base, &shoulder, &elbow, &wrist, &wristRot, &gripper);

  pinMode(A0, INPUT);  // Joystick 1 X
  pinMode(A1, INPUT);  // Joystick 1 Y
  pinMode(A2, INPUT);  // Joystick 2 X
  pinMode(A3, INPUT);  // Joystick 2 Y
  pinMode(7, OUTPUT);  // Relay

  pinMode(2, INPUT);
  pinMode(8, INPUT);

  digitalWrite(7, LOW);

  digitalWrite(2, HIGH);
  digitalWrite(8, HIGH);

  delay(10);

  myrobot.moveJoints(90, 90, 90, 90, 90, 100);

  delay(200);
}

void loop() {
  myrobot.moveWithJoystick(A0, A1, 2, A2, A3, 8);
  delay(TIME);
}


// vim:filetype=cpp
// vim:filetype=arduino
