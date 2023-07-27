#include <Stepper.h>

const int stepsPerRevolution1 = 2048;    //stepsPerRevolution
const int stepsPerRevolution2 = 2048;
const int stepsPerRevolution3 = 2048;
const int stepsPerRevolution4 = 2048;
const int stepsPerRevolution5 = 2048;

Stepper Stepper1(stepsPerRevolution1, 22, 24, 23, 25);
Stepper Stepper2(stepsPerRevolution2, 26, 28, 27, 29);
Stepper Stepper3(stepsPerRevolution3, 30, 32, 31, 33);
Stepper Stepper4(stepsPerRevolution4, 34, 36, 35, 37);
Stepper Stepper5(stepsPerRevolution5, 38, 40, 39, 41);

int stepCount = 0;
const int X1_pin = 0;
const int Y1_pin = 1;
const int X2_pin = 2;
const int Y2_pin = 3;
const int Pote_pin =4;

int Stpper_pos = 30;
int RPM;

void setup() {
  Serial.begin(9600);
}

void loop() {
  /*Stepper1.setSpeed(RPM);
  pot = analogRead(3);
  RPM = map (pot, 0, 1023, 4, 18);
  Serial.print("RPM: ");
  Serial.println(RPM);
  */
  /*X_pos = analogRead(X1_pin);
  
  if (X_pos > 600) {
  Stepper1.step(Stpper_pos);
  Serial.print("    steps: ");
  Serial.print(stepCount);
  Serial.println(   X_pos);
  stepCount++;
  //delay(dt);
  }

  if (X_pos < 400) {
  Stepper1.step(-Stpper_pos);
  Serial.print("    steps:");
  Serial.print(stepCount);
  Serial.println(   X_pos);
  stepCount--;
  //delay(dt);
  }
  //delay (100);*/
}

void RPM_val() {
  RPM = map (analogRead(Pote_pin), 0, 1023, 4, 18);
  Serial.print("RPM: ");
  Serial.println(RPM);
  Stepper1.setSpeed(RPM);
  Stepper2.setSpeed(RPM);
  Stepper3.setSpeed(RPM);
  Stepper4.setSpeed(RPM);
  Stepper5.setSpeed(RPM);
}

void stepper1() {
  if (analogRead(X1_pin) > 600) {
  Stepper1.step(Stpper_pos);
  }
  if (analogRead(X1_pin) < 400) {
  Stepper1.step(-Stpper_pos);
  }
}
void stepper2() {  
  if (analogRead(Y1_pin) > 600) {
  Stepper1.step(Stpper_pos);
  }
  if (analogRead(Y1_pin) < 400) {
  Stepper1.step(-Stpper_pos);
  }  
}
void stepper3() {
  if (analogRead(X2_pin) > 600) {
  Stepper1.step(Stpper_pos);
  }
  if (analogRead(X2_pin) < 400) {
  Stepper1.step(-Stpper_pos);
  }
}
void stepper4() {  
  if (analogRead(Y2_pin) > 600) {
  Stepper1.step(Stpper_pos);
  }
  if (analogRead(Y2_pin) < 400) {
  Stepper1.step(-Stpper_pos);
  }  
}
