#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define BASE        0
#define SHOULDER    1
#define ELBOW       2
#define WRIST_ROT   3
#define WRIST       4
#define GRIPPER     5

#define BAS_MAX     2400
#define BAS_MIN     450

#define SHL_MAX     2000
#define SHL_MIN     750

#define ELB_MAX     2050     
#define ELB_MIN     630

#define WROT_MAX    2150
#define WROT_MIN    630

#define WRI_MAX     2150
#define WRI_min     630

#define GRI_MAX     2150
#define GRI_min     630



int pos1 = 1500;
int pos2 = 1390;
int pos3 = 1300;
int pos4 = 1300;
int pos5 = 1300;
int dt=25;
int steps = 5;

void setup() {
  Serial.begin(115200);
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  
  pinMode(A0,INPUT);      // Joystick 1 X
  pinMode(A1,INPUT);      // Joystick 1 Y
  pinMode(A2,INPUT);      // Joystick 2 X
  pinMode(A3,INPUT);      // Joystick 2 Y
  pinMode(7, OUTPUT);     // Relay
  
  pinMode (2,INPUT);
  pinMode (8,INPUT);

  digitalWrite(7, LOW);

  //digitalWrite(butten1, HIGH);  
  //digitalWrite(butten2, HIGH);

  delay(10);
  
  pwm.writeMicroseconds(BASE, pos1);        
  pwm.writeMicroseconds(SHOULDER, pos2);      
  pwm.writeMicroseconds(ELBOW, pos3);      
  pwm.writeMicroseconds(WRIST_ROT, pos4);      
  pwm.writeMicroseconds(WRIST, pos5);  

  delay(200);
  
}

void loop() {
  int x1_val = analogRead(A0);
  int x2_val = analogRead(A2);
  int y1_val = analogRead(A1);
  int y2_val = analogRead(A3);

  //Serial.println(pos5);
  //delay(200);
  
  if (y1_val > 800) {
    if (pos1 < BAS_MAX) {
      pwm.writeMicroseconds(BASE, pos1);  
      pos1+=steps;
      //Serial.println(pos1);
      delay(dt);
    }

  }
  if (y1_val < 400) {
    if (pos1 > BAS_MIN) {
      pwm.writeMicroseconds(BASE, pos1);  
      pos1-=steps;
      //Serial.println(pos1);
      delay(dt);
    }
  }

  
  if (x1_val > 800) {
    if (pos2 < SHL_MAX) {
      pwm.writeMicroseconds(SHOULDER, pos2);  
      pos2+= steps;
      //Serial.println(pos2);
      delay(dt);
    }

  }
  if (x1_val < 400) {
    if (pos2 > SHL_MIN) {
      pwm.writeMicroseconds(SHOULDER, pos2);  
      pos2-=steps;
      //Serial.println(pos2);
      delay(dt);
    }
  }


  if (x2_val > 800) {
    if (pos3 < ELB_MAX) {
      pwm.writeMicroseconds(ELBOW, pos3);  
      pos3+=steps;
      //Serial.println(pos3);
      delay(dt);
    }
  }
  if (x2_val < 400) {
    if (pos3 > ELB_MIN) {
      pwm.writeMicroseconds(ELBOW, pos3);  
      pos3-=steps;
      //Serial.println(pos3);
      delay(dt);
    }
  }

  
  if (y2_val > 800) {
    if (pos4 < WROT_MAX) {
      pwm.writeMicroseconds(WRIST_ROT, pos4);  
      pos4+=steps;
      //Serial.println(pos4);
      delay(dt);
    }

  }
  if (y2_val < 400) {
    if (pos4 > WROT_MIN) {
      pwm.writeMicroseconds(WRIST_ROT, pos4);  
      pos4-=steps;
      //Serial.println(pos4);
      delay(dt);
    }
  }


  if (digitalRead(2) == HIGH && digitalRead(8)== LOW) {
    if (pos5 < WRI_MAX) {
      pwm.writeMicroseconds(WRIST, pos5);  
      pos5+=steps;
      //Serial.println(pos5);
      delay(dt);
    }
  }
  
  if (digitalRead(8) == HIGH && digitalRead(2)== LOW) {
    if (pos5 > WRI_MAX) {
      pwm.writeMicroseconds(WRIST, pos5);  
      pos5-=steps;
      //Serial.println(pos5);
      delay(dt);
    }
  }
}

