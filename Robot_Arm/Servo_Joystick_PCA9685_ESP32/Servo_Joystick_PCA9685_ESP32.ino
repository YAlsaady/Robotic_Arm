#include "Joint.hh"
#include "Robot.hh"
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
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

LiquidCrystal_I2C lcd(0x27, 20, 4);


void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(10);

  lcd.init();
  lcd.backlight();

  base.setJoint    (     0,      600,        2400,       1550,       STEPS       );
  shoulder.setJoint(     1,      950,        1550,       1550,       STEPS / 5   );
  elbow.setJoint   (     2,      780,        1940,       1360,       STEPS / 2   );
  wrist.setJoint   (     4,      570,        2340,       1455,       STEPS       );
  wristRot.setJoint(     3,      450,        2220,       1350,       STEPS       );
  gripper.setJoint (     5,      630,        2150,       1300,       STEPS       );

  myrobot.setRobot(&base, &shoulder, &elbow, &wrist, &wristRot, &gripper);
  myrobot.setJoystick(34, 35, 32, 36, 39, 33, 4000, 800);
  myrobot.setDimension(139, 226, 226, 60);

  pinMode(25, OUTPUT); // Relay
  digitalWrite(25, LOW);

  // myrobot.moveJoints(90, 90, 90, 90, 90, 0);
  // myrobot.moveEndEffector(0, 300, 100, -90);
  delay(200);
}

void loop() {
  static unsigned long lastUpdateTime = 0;
  static const unsigned long updateInterval =1000;
  unsigned long currentTime = millis();

  if (currentTime - lastUpdateTime >= updateInterval) {
    myrobot.lcdPrint();
    lastUpdateTime = currentTime;
  }

  // myrobot.moveEndEffector_Demo();
  // myrobot.moveWithJoystick();
  myrobot.moveEndEffector_Joystick();
}

// vim:filetype=cpp
// vim:filetype=arduino
