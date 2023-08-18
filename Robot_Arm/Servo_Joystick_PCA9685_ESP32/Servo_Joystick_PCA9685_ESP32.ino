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

LiquidCrystal_I2C lcd(0x27, 20, 4);
ESP32Encoder encoder;

bool isSelected = false;

void IRAM_ATTR isr() {
  static unsigned long lastUpdateTime = 0;
  if (millis() - lastUpdateTime < 200) {
    return;
  }
  isSelected = true;
  lastUpdateTime = millis();
}

void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(10);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Robotic Arm Menu:");

  base.setJoint    (     0,      600,        2400,       1550,       STEPS       );
  shoulder.setJoint(     1,      950,        1550,       1550,       STEPS / 5   );
  elbow.setJoint   (     2,      780,        1940,       1360,       STEPS / 2   );
  wrist.setJoint   (     4,      570,        2340,       1455,       STEPS       );
  wristRot.setJoint(     3,      450,        2220,       1350,       STEPS       );
  gripper.setJoint (     5,      630,        2150,       1300,       STEPS       );

  myrobot.setRobot(&base, &shoulder, &elbow, &wrist, &wristRot, &gripper);
  myrobot.setJoystick(35, 34, 32, 39, 36, 33, 4000, 800);
  myrobot.setDimension(139, 226, 226, 60);

  pinMode(25, OUTPUT); // Relay
  digitalWrite(25, LOW);

  encoder.attachHalfQuad(27, 26);
  encoder.setCount(0);
  attachInterrupt(digitalPinToInterrupt(14), isr, RISING);

  // myrobot.moveJoints(90, 90, 90, 90, 90, 0);
  // myrobot.moveEndEffector(0, 300, 100, -90);
  delay(200);
}

void loop() {
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
    option = "Joystick:Endeffector";
    break;
  case 1:
    option = "Joystick:Joints     ";
    break;
  case 2:
    option = "Demo                ";
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
    lcd.setCursor(0, 1);
    lcd.print(option);
    myrobot.lcdPrint();
    lastUpdateTime = millis();
  }
}

// vim:filetype=cpp
// vim:filetype=arduino
