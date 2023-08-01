#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Joint.hh"
#include "Robot.hh"

#define TIME 10
#define STEPS 5

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

joint base    (     0,      450,        2400,       1500,       STEPS       );
joint shoulder(     1,      750,        2000,       1360,       STEPS / 2   );
joint elbow   (     2,      630,        2050,       1340,       STEPS       );
joint wrist   (     4,      630,        2150,       1500,       STEPS / 2   );
joint wristRot(     3,      450,        2400,       1350,       STEPS       );
joint gripper (     5,      630,        2150,       1300,       STEPS       );

Robot myrobot(&base, &shoulder, &elbow, &wrist, &wristRot, &gripper);

void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

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

  base.move();
  shoulder.move();
  elbow.move();
  wrist.move();
  wristRot.move();
  gripper.move();

  delay(200);
}

void loop() {
  myrobot.moveWithJoystick(A0, A1, 2, A2, A3, 8);
  /*
  int x1_val = analogRead(A0);
  int x2_val = analogRead(A2);
  int y1_val = analogRead(A1);
  int y2_val = analogRead(A3);

  if (y1_val > 800) {
    base.moveSteps(HIGH);
  }
  if (y1_val < 400) {
    base.moveSteps(LOW);
  }

  if (x1_val > 800) {
    shoulder.moveSteps(HIGH);
  }
  if (x1_val < 400) {
    shoulder.moveSteps(LOW);
  }

  if (x2_val > 800) {
    elbow.moveSteps(HIGH);
  }
  if (x2_val < 400) {
    elbow.moveSteps(LOW);
  }

  if (y2_val > 800) {
    wrist.moveSteps(HIGH);
  }
  if (y2_val < 400) {
    wrist.moveSteps(LOW);
  }

  if (digitalRead(2) == HIGH && digitalRead(8) == LOW) {
    wristRot.moveSteps(HIGH);
  }
  if (digitalRead(2) == LOW && digitalRead(8) == HIGH) {
    wristRot.moveSteps(LOW);
  }

  Serial.println(shoulder.position());
  delay(TIME);
  */
  delay(TIME);
  //Serial.println(analogRead(A0)); delay(200);
}
