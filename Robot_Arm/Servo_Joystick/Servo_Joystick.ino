#include <Servo.h>
Servo servo1;
Servo servo2;

Servo servo3;
Servo servo4;

Servo servo5;

int stepCount1 = 0;
int X1_pin=A0;
int xservo1 = 3;
int pos1 = 90;

int stepCount2 = 0;
int X2_pin=A1;
int yservo1 = 5;
int pos2 = 90;



int stepCount3 = 0;
int X3_pin=A2;
int xservo2 = 6;
int pos3 = 90;

int stepCount4 = 0;
int X4_pin=A3;
int yservo2 = 9;
int pos4 = 90;


int butten1=2;
int butten2=7;
int pos5 = 90;

int dt=25;


void setup() {
  pos1 = pos2 = pos3 = pos4 = pos5 = 90;
  //Serial.begin(9600);
  pinMode(X1_pin,INPUT);
  pinMode(X2_pin,INPUT);
  pinMode(3,OUTPUT);
  pinMode(5,OUTPUT);

  pinMode(X3_pin,INPUT);
  pinMode(X4_pin,INPUT);
  pinMode(6,OUTPUT);
  pinMode(9,OUTPUT);

  pinMode (butten1,INPUT);
  digitalWrite(butten1, HIGH);  
  pinMode (butten2,INPUT);
  digitalWrite(butten2, HIGH);  


  servo1.attach(3);
  servo2.attach(5);
  servo1.write(pos1);
  servo2.write(pos2);

  servo3.attach(6);
  servo4.attach(9);
  servo3.write(pos3);
  servo4.write(pos4);

  servo5.attach(10);
  servo5.write(pos5);
}

void loop() {
  
  if (analogRead(X1_pin) > 800) {
    if (pos1 < 180) {
      servo1.write(pos1);
      pos1++;
      //Serial.println(pos1);
      delay(dt);
    }

  }
  if (analogRead(X1_pin) < 400) {
    if (pos1 > 0) {
      servo1.write(pos1);
      pos1--;
      //Serial.println(pos1);
      delay(dt);
    }
  }
  if (analogRead(X2_pin) > 800) {
    if (pos2 < 180) {
      servo2.write(pos2);
      pos2++;
      //Serial.println(pos2);
      delay(dt);
    }

  }
  if (analogRead(X2_pin) < 400) {
    if (pos2 > 0) {
      servo2.write(pos2);
      pos2--;
      //Serial.println(pos2);
      delay(dt);
    }
  }


   if (analogRead(X3_pin) > 900) {
    if (pos3 < 180) {
      servo3.write(pos3);
      pos3++;
      //Serial.println(pos3);
      delay(dt);
    }

  }
  if (analogRead(X3_pin) < 300) {
    if (pos3 > 0) {
      servo3.write(pos3);
      pos3--;
      Serial.println(pos3);
      delay(dt);
    }
  }
  if (analogRead(X4_pin) > 900) {
    if (pos4 < 180) {
      servo4.write(pos4);
      pos4++;
      //Serial.println(pos4);
      delay(dt);
    }

  }
  if (analogRead(X4_pin) < 300) {
    if (pos4 > 0) {
      servo4.write(pos4);
      pos4--;
      Serial.println(pos4);
      delay(dt);
    }
  }


  if (digitalRead(butten1)==LOW) {
    if (pos5 < 180) {
      servo5.write(pos5);
      pos5++;
      //Serial.println(pos5);
      delay(dt);
    }
  }
  
  if (digitalRead(butten2)==LOW) {
    if (pos5 > 0) {
      servo5.write(pos5);
      pos5--;
      //Serial.println(pos5);
      delay(dt);
    }
  }
}
