#include <Servo.h>          //standard library for the servo
#include <NewPing.h>        //for the Ultrasonic sensor function library.
#include <Wire.h>
//L298N motor control pins
#define RightMotorForward  4
#define RightMotorBackward  7
#define LeftMotorForward  2
#define LeftMotorBackward  3
#define SCleft_motor  5 // speed control using pwm pins connected to spped control pin on l293d
#define SCright_motor  6// speed control using pwm pins connected to spped control pin on l293d
#define pushton 10 // a push button to turn off/on auto-pilot
#define trig_pin 9  // Ultra Sonic Sensor
#define echo_pin 8
#define servoPin 11
const int maximum_distance = 250;
int turn_angleL = 90; // can be adjusted by console by "P<value>"
int turn_angleR = 90; // can be adjusted by console by "p<value>"
int movespeed = 160; // can be adjusted by console by "h<value>"
int distance = 100;
boolean goesForward = false;
boolean buttonFlag = false;
String temp;
char inChar;
const int MPU = 0x68; // MPU6050 I2C address
float GyroX, GyroY, GyroZ;
float elapsedTime, currentTime, previousTime;
float inyaw, yaw;
NewPing sonar(trig_pin, echo_pin, maximum_distance); //sensor function
Servo servo_motor;



void setup() {
  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  pinMode(pushton, INPUT_PULLUP);


  servo_motor.attach(servoPin);
  servo_motor.write(90);
  delay(1000);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  //Setup usb serial connection to computer
  Wire.begin();
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  Serial.begin(9600);


}

void loop() {
  if (!digitalRead(pushton) and buttonFlag) {
    buttonFlag = false;
    moveStop();
    delay(200);
  }
  else if (!digitalRead(pushton) and !buttonFlag) {
    buttonFlag = true;
    delay(200);
  }
  int distanceRight = 0;
  int distanceLeft = 0;
  while (Serial.available()) {
    inChar = (char)Serial.read();
    Serial.println(inChar);
  }

  // used for bluetooth remote that sends the following characters to control the car over network if needed
  switch (inChar) {
    case 'w':
      moveStop();
      moveForward();
      break;
    case 'a':
      turnLeft();
      break;
    case 's':
      moveBackward();
      break;
    case 'd':
      turnRight();
      break;
    case 'k':
      moveStop();
      break;
    case 'h': // Used to set the speed of the car (0-100)
      inChar = ' ';
      temp = Serial.readString(); //Reading the Input string from Serial port.
      movespeed = map(temp.toInt(), 100, 0, 255, 0);
      Serial.println(movespeed);
      analogWrite(SCleft_motor, movespeed);
      analogWrite(SCright_motor, movespeed);
      break;
    case 'P': // Used to set left turn angle, default 90* measured by on car gyro sensor
      inChar = ' ';
      temp = Serial.readString(); //Reading the Input string from Serial port.
      turn_angleL = temp.toInt();
      Serial.println(turn_angleL);
      break;
    case 'p': // Used to set left turn angle, default 90* measured by on car gyro sensor
      inChar = ' ';
      temp = Serial.readString(); //Reading the Input string from Serial port.
      turn_angleR = temp.toInt();
      Serial.println(turn_angleR);
      break;

    case 'f': // used to turn auto pilot on/off
      if (buttonFlag) {
        buttonFlag = false;
        moveStop();
      }
      else {
        buttonFlag = true;
      }
      break;

  }
  if (buttonFlag) {
    if (distance <= 26) {
      moveStop();
      delay(300);
      moveBackward();
      delay(900);
      moveStop();
      delay(300);
      distanceRight = lookRight();
      delay(800);
      distanceLeft = lookLeft();
      delay(300);

      if (distanceRight >= distanceLeft) {
        turnRight();
        moveStop();
      }
      else {
        turnLeft();
        moveStop();
      }

    }
    else {
      moveForward();
    }
    distance = readPing();
  }
}
int lookRight() {
  servo_motor.write(0);
  delay(500);
  int distance = readPing();
  delay(300);
  servo_motor.write(90);
  return distance;
}

int lookLeft() {
  servo_motor.write(180);
  delay(500);
  int distance = readPing();
  delay(300);
  servo_motor.write(90);
  return distance;
  delay(100);
}

int readPing() {
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = 250;
  }
  return cm;
}

void moveStop() {
  analogWrite(SCleft_motor, 0);
  analogWrite(SCright_motor, 0);
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
  goesForward = false;


}

void moveForward() {

  if (!goesForward) {
    goesForward = true;
    normalspeed();
    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorForward, HIGH);

    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorBackward, LOW);

  }
}

void moveBackward() {

  goesForward = false;
  maxspeed();
  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);

  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorForward, LOW);


}

void turnRight() {
  custSpeed(180);
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);

  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);

  inyaw=zrot();
  while (fabs(zrot()-inyaw) < turn_angleR) {} // Stalls the code until the car takes full 90* right turn
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorForward, LOW);

  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);



}

void turnLeft() {
  custSpeed(180);
  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, HIGH);

  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);

  inyaw=zrot();
  while (fabs(zrot()-inyaw) < turn_angleL) {} // Stalls the code until the car takes full 90* left turn
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorForward, LOW);

  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
}

void maxspeed() {
  analogWrite(SCleft_motor, 255);
  analogWrite(SCright_motor, 255);
}
void normalspeed() {
  analogWrite(SCleft_motor, movespeed);
  analogWrite(SCright_motor, movespeed);
}

void custSpeed( int cuSpeed ){
  analogWrite(SCleft_motor, cuSpeed);
  analogWrite(SCright_motor, cuSpeed);
}
float zrot(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers

  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  yaw =  (yaw + GyroZ * elapsedTime);
  return yaw;}
