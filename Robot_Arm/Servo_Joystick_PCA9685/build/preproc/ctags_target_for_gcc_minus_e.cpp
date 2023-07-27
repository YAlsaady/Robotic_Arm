# 1 "/home/yaman/Git/Arduino/Robot_Arm/Servo_Joystick_PCA9685/Servo_Joystick_PCA9685.ino"
# 2 "/home/yaman/Git/Arduino/Robot_Arm/Servo_Joystick_PCA9685/Servo_Joystick_PCA9685.ino" 2
# 3 "/home/yaman/Git/Arduino/Robot_Arm/Servo_Joystick_PCA9685/Servo_Joystick_PCA9685.ino" 2

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
# 33 "/home/yaman/Git/Arduino/Robot_Arm/Servo_Joystick_PCA9685/Servo_Joystick_PCA9685.ino"
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
  pwm.setPWMFreq(50); // Analog servos run at ~50 Hz updates

  pinMode(A0,0x0); // Joystick 1 X
  pinMode(A1,0x0); // Joystick 1 Y
  pinMode(A2,0x0); // Joystick 2 X
  pinMode(A3,0x0); // Joystick 2 Y
  pinMode(7, 0x1); // Relay

  pinMode (2,0x0);
  pinMode (8,0x0);

  digitalWrite(7, 0x0);

  //digitalWrite(butten1, HIGH);  
  //digitalWrite(butten2, HIGH);

  delay(10);

  pwm.writeMicroseconds(0, pos1);
  pwm.writeMicroseconds(1, pos2);
  pwm.writeMicroseconds(2, pos3);
  pwm.writeMicroseconds(3, pos4);
  pwm.writeMicroseconds(4, pos5);

  delay(200);

}

void loop() {
  int x1_val = analogRead(A0);
  int x2_val = analogRead(A2);
  int y1_val = analogRead(A1);
  int y2_val = analogRead(A3);

  //Serial.println(pos5);
  //delay(200);

  if (x1_val > 800) {
    if (pos1 < 2400) {
      pwm.writeMicroseconds(0, pos1);
      pos1+=steps;
      //Serial.println(pos1);
      delay(dt);
    }

  }
  if (x1_val < 400) {
    if (pos1 > 450) {
      pwm.writeMicroseconds(0, pos1);
      pos1-=steps;
      //Serial.println(pos1);
      delay(dt);
    }
  }


  if (x2_val > 800) {
    if (pos2 < 2000) {
      pwm.writeMicroseconds(1, pos2);
      pos2+= steps;
      //Serial.println(pos2);
      delay(dt);
    }

  }
  if (x2_val < 400) {
    if (pos2 > 750) {
      pwm.writeMicroseconds(1, pos2);
      pos2-=steps;
      //Serial.println(pos2);
      delay(dt);
    }
  }


  if (y1_val > 800) {
    if (pos3 < 2050) {
      pwm.writeMicroseconds(2, pos3);
      pos3+=steps;
      //Serial.println(pos3);
      delay(dt);
    }
  }
  if (y1_val < 400) {
    if (pos3 > 630) {
      pwm.writeMicroseconds(2, pos3);
      pos3-=steps;
      //Serial.println(pos3);
      delay(dt);
    }
  }


  if (y2_val > 800) {
    if (pos4 < 2150) {
      pwm.writeMicroseconds(3, pos4);
      pos4+=steps;
      //Serial.println(pos4);
      delay(dt);
    }

  }
  if (y2_val < 400) {
    if (pos4 > 630) {
      pwm.writeMicroseconds(3, pos4);
      pos4-=steps;
      //Serial.println(pos4);
      delay(dt);
    }
  }


  if (digitalRead(2) == 0x1 && digitalRead(8)== 0x0) {
    if (pos5 < 2150) {
      pwm.writeMicroseconds(4, pos5);
      pos5+=steps;
      //Serial.println(pos5);
      delay(dt);
    }
  }

  if (digitalRead(8) == 0x1 && digitalRead(2)== 0x0) {
    if (pos5 > 2150) {
      pwm.writeMicroseconds(4, pos5);
      pos5-=steps;
      //Serial.println(pos5);
      delay(dt);
    }
  }
}
