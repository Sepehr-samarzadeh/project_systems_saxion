#include <Arduino.h>

#include <Servo.h>

#define hardleftSensorPin A0
#define middleSensorPin A1
#define hardrightSensorPin A2
#define leftSensorPin A3
#define rightSensorPin A4
#define motorPin 9

// Motor Connections (Both must use PWM pins)
#define RPWM 5
#define LPWM 6
#define maxSpeed 30

Servo myServo;
int servoPin = 3;
int servoPos = 90;

int leftSensorValue, middleSensorValue, rightSensorValue, hardleftSensorValue, hardrightSensorValue;

// flag to track if the white line is detected
bool whiteLineDetected = false;

//milies funcionality
unsigned long currentMillis = 0;
unsigned long accePreviousMillis = 0;
unsigned long decePreviousMillis = 0;
unsigned long caliWitPrviousMillis = 0;
unsigned long caliBlkPrviousMillis = 0;
unsigned long leftTurnMillis = 0;
unsigned long rightTurnMillis = 0;
unsigned long stoppingMillis = 0;

//function periods for delays and syncing
const long motorRevDelay = 20;
const long turnDelay = 50;
const long stopDelay = 1500;
const long calibrattime = 1500;

int currentSpeed = 0;

enum State {
  straight,
  turn_left,
  turn_right,
  stop
};


void setup() {
  Serial.begin(9600);
  pinMode(motorPin, OUTPUT);
  // calibrateSensors();
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  pinMode(A4,INPUT);
  //servo
  myServo.attach(servoPin);
  myServo.write(servoPos);

   // Set motor connections as outputs
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);

  // Stop motors
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  myServo.write(servoPos);
}

void loop() {
  currentMillis = millis();
  
  leftSensorValue = !digitalRead(leftSensorPin);
  middleSensorValue = !digitalRead(middleSensorPin);
  rightSensorValue = !digitalRead(rightSensorPin);
  hardleftSensorValue = !digitalRead(hardleftSensorPin);
  hardrightSensorValue = !digitalRead(hardrightSensorPin);

  //Serial.print("Left Sensor: ");
  //Serial.print(leftSensorValue);
  //Serial.print("\tMiddle Sensor: ");
  //Serial.print(middleSensorValue);
  //Serial.print("\tRight Sensor: ");
  //Serial.println(rightSensorValue);
 

  // follow the line based on the sensor readings
  followLine();
}

void followLine() {
  bool lineDetected = false;
  static bool hardTurn = false;
  static State current_state = straight;


  //drive if any of the sensors output high
  if(current_state != stop)
  {
   currentSpeed = accelerate(currentSpeed);
  }


  switch (current_state)
  {
    case straight:
    //driving functionality
    steerStraight();
    //state switching
      if (hardleftSensorValue == 1 && middleSensorValue == 1 && hardrightSensorValue == 1) current_state = stop;
      else if (rightSensorValue == 1 || hardrightSensorValue == 1) current_state = turn_right;
      else if (leftSensorValue == 1 || hardleftSensorValue == 1) current_state = turn_left;
      break;
    case turn_right:
      //turn_right functionality
      if (hardrightSensorValue == 1) hardTurn = true;
      if (rightSensorValue == 1) hardTurn = false;
      if (hardTurn) steerhardRight();
      else steerRight();

      //state switching
      if (hardleftSensorValue == 1 && middleSensorValue == 1 && hardrightSensorValue == 1) current_state = stop;
      else if (middleSensorValue == 1) current_state = straight;
      else if (leftSensorValue == 1 || hardleftSensorValue) current_state = turn_left;
      break;
    case turn_left:
      //turn_left functionality
      if (hardleftSensorValue == 1) hardTurn = true;
      if (leftSensorValue == 1) hardTurn = false;
      if (hardTurn) steerhardLeft();
      else steerLeft();

      //state switching
      if (hardleftSensorValue == 1 && middleSensorValue == 1 && hardrightSensorValue == 1) current_state = stop;
      else if (middleSensorValue == 1) current_state = straight;
      else if (rightSensorValue == 1 || hardrightSensorValue) current_state = turn_right;
      break;
    case stop:
      currentSpeed = decelerate(currentSpeed);
      if (hardleftSensorValue == 0 && leftSensorValue == 0 && middleSensorValue == 1 && rightSensorValue == 0 && hardrightSensorValue == 0) current_state = straight;
      break;
  }

}

void steerRight(){
 
  Serial.println("turning right");
    Serial.println(servoPos);
    if(servoPos >80){
      servoPos = 81;
      myServo.write(servoPos);
      delay(100);
      servoPos = 90;
      myServo.write(servoPos);
    } 
     Serial.println(servoPos);
}
void steerhardRight(){
 
  Serial.println("turning hard right");
    Serial.println(servoPos);
    if(servoPos >30){
      servoPos = 31;
      myServo.write(servoPos);
      delay(100);
      servoPos = 90;
      myServo.write(servoPos);
    } 
     Serial.println(servoPos);
}
void steerStraight(){
 
  Serial.println("turning straight");
    Serial.println(servoPos);
    if(servoPos >91 || servoPos < 90){
      servoPos = 90;
      myServo.write(servoPos);
      delay(100);
      servoPos = 90;
      myServo.write(servoPos);
    } 
     Serial.println(servoPos);
}
void steerLeft(){
  Serial.println("turning left");
    //Serial.println(servoPos);
    if(servoPos <100){
      servoPos = 101;
      myServo.write(servoPos);
      delay(100);
      servoPos = 90;
      myServo.write(servoPos);
    } 
     Serial.println(servoPos);
}
void steerhardLeft(){
  Serial.println("turning hard left");
    Serial.println(servoPos);
    if(servoPos <140){
      servoPos = 139;
      myServo.write(servoPos);
      delay(100);
      servoPos = 90;
      myServo.write(servoPos);
    } 
     Serial.println(servoPos);
}


int accelerate(int currentSpeed){
  // if(currentMillies - accePreviousMillies >= motorRevDelay ){
  //   accePreviousMillies = currentMillies;
    digitalWrite(RPWM, LOW);
    for (int i = currentSpeed; i < maxSpeed; i++) {
      analogWrite(LPWM, i);
    }
  // }
  return currentSpeed;
}

int decelerate(int currentSpeed){
  // if(currentMillies - decePreviousMillies >= motorRevDelay){
  //   decePreviousMillies = currentMillies;
    digitalWrite(RPWM, LOW);
    for (int i = currentSpeed; i >= 0; i--) {
      analogWrite(LPWM, 0);
    }
  // }
  return currentSpeed;
}

//   double error = digitalRead(leftPin) && digitalRead(rightPin);
  

//  double pid(double error){
//   double P = error; //proportional
//   static double integral += error * dt; // change dt to angle maybe
//   double derivative = (error - pre) / dt; 
//   pre = error;
//   doulbe output = (kp * P ) + (ki * integral) + (kd * derivative);
//   return output;
//  }