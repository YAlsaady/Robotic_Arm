#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

class Joint {
private:
  byte pinNum;
  unsigned maxVal;
  unsigned minVal;
  unsigned pos;
  unsigned steps;
public:
  Joint();
  Joint(byte pinNum, unsigned minVal, unsigned maxVal, unsigned pos, unsigned startPos) {
    this->pinNum = pinNum;
    this->maxVal = maxVal;
    this->minVal = minVal;
    this->pos = pos;
    this->steps = startPos;
  }

  void move() {
    pwm.writeMicroseconds(this->pinNum, this->pos);
  }

  void moveMs(unsigned time) {
    this->pos = time;
    pwm.writeMicroseconds(this->pinNum, this->pos);
  }

  void moveDegree(int angle) {
    this->pos = map(angle, maxVal, minVal, 0, 180);
    moveMs(this->pos);
  }

  void moveSteps(bool direction) {
    if (direction == HIGH && (this->pos < this->maxVal)) {
      this->pos += this->steps;
    } else if (direction == LOW && (this->pos > this->minVal)) {
      this->pos -= this->steps;
    }
    move();
  }

  unsigned position() {
    return pos;
  }
};


int dt = 10;
int steps = 5;

Joint base(0, 450, 2400, 1500, steps);
Joint shoulder(1, 750, 2000, 1390, (steps / 2));
Joint elbow(2, 630, 2050, 1300, steps);
Joint wrist(3, 630, 2150, 1300, (steps / 2));
Joint wristRot(4, 450, 2400, 1300, steps);
Joint gripper(5, 630, 2150, 1300, steps);

void setup() {
  Serial.begin(115200);

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
  int x1_val = analogRead(A0);
  int x2_val = analogRead(A2);
  int y1_val = analogRead(A1);
  int y2_val = analogRead(A3);

  //Serial.println(digitalRead(2));
  //delay(200);

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
  delay(dt);
}
