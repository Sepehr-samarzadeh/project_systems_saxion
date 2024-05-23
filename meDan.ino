#include <Servo.h>
#define leftSensorPin A0
#define middleSensorPin A1
#define rightSensorPin A2
#define motorPin 9

// Motor Connections (Both must use PWM pins)
#define RPWM 5
#define LPWM 6
#define maxSpeed 30

Servo myServo;
int servoPin = 3;
int servoPos = 90;

int leftSensorValue, middleSensorValue, rightSensorValue;
int blackThreshold, whiteThreshold;

// flag to track if the white line is detected
bool whiteLineDetected = false;

//milies funcionality
unsigned long currentMillis = 0;
unsigned long accePreviousMillis = 0;
unsigned long decePreviousMillis = 0;
unsigned long leftTurnMillis = 0;
unsigned long rightTurnMillis = 0;
unsigned long stoppingMillis = 0;

//function periods for delays and syncing
const long motorRevDelay = 20;
const long turnDelay = 100;
const long stopDelay = 1500;

int currentSpeed = 0;


void setup() {
  Serial.begin(9600);
  pinMode(motorPin, OUTPUT);
  calibrateSensors();
  
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
  
  leftSensorValue = analogRead(leftSensorPin);
  middleSensorValue = analogRead(middleSensorPin);
  rightSensorValue = analogRead(rightSensorPin);

  Serial.print("Left Sensor: ");
  Serial.print(leftSensorValue);
  Serial.print("\tMiddle Sensor: ");
  Serial.print(middleSensorValue);
  Serial.print("\tRight Sensor: ");
  Serial.println(rightSensorValue);

  // follow the line based on the sensor readings
  whiteLineDetected = followLine();

  if (!whiteLineDetected) {
    Serial.println("No white line detected. Stopping the car.");
    currentSpeed = decelerate(currentSpeed);
  }

  // delay(1000);
}

bool followLine() {
  bool lineDetected = false;
  static bool state = false;

  if (leftSensorValue < whiteThreshold) {
    // left sensor detects the white line
    state = true;
    static bool turn;
    Serial.println("turning left");
    currentSpeed = accelerate(currentSpeed);
    Serial.println(servoPos);
    if(servoPos <110){
      servoPos+=10;
      moveLeft(servoPos);
      leftTurnMillis = currentMillis;
    }

    if(currentMillis - leftTurnMillis >= turnDelay ){
     leftTurnMillis = currentMillis;
     myServo.write(90);
    }
    
    lineDetected = true;
  } else if (rightSensorValue < whiteThreshold) {
    state = true;
    Serial.println("turning right");
    currentSpeed = accelerate(currentSpeed);
    if(servoPos >65){
      servoPos-=5;
      moveRight(servoPos);
      rightTurnMillis = currentMillis;
    }
    if(currentMillis - rightTurnMillis >= turnDelay ){
     rightTurnMillis = currentMillis;
     myServo.write(90);
    }

    lineDetected = true;
  } else if (middleSensorValue < whiteThreshold) {
    state = false;
    Serial.println("moving forward");
    currentSpeed = accelerate(currentSpeed);
    lineDetected = true;
  } else if (leftSensorValue > blackThreshold && middleSensorValue > blackThreshold && rightSensorValue > blackThreshold) {
    // no sensor detects the white line
    stoppingMillis = currentMillis;
      if (currentMillis - stoppingMillis >= stopDelay && state == false ){
        stoppingMillis = currentMillis;
        state = true;
        Serial.println("Stopping!");
        currentSpeed = decelerate(currentSpeed);
      }
      Serial.println("no line detected");
 
    
    
  }

  return lineDetected;
}

void moveRight(int num){
	
  myServo.write(num);
  Serial.println(num);
}
void moveLeft(int num){
  myServo.write(num);
  Serial.println(num);
}



void calibrateSensors() {
  Serial.println("starting calibration...");
  Serial.println("place the sensors over the black surface");

  // read the sensor values over the black surface
  int blackReadings[3];
  for (int i = 0; i < 3; i++) {
    blackReadings[i] = analogRead(i == 0 ? leftSensorPin : (i == 1 ? middleSensorPin : rightSensorPin));
    delay(100);
  }
  // find the max value for black anything more than this value count as black
  blackThreshold = max(max(blackReadings[0], blackReadings[1]), blackReadings[2]) - 50;

  Serial.print("black threshold: ");
  Serial.println(blackThreshold);

  Serial.println("place the sensors over the white line.");

  // read the sensor values over the white line
  int whiteReadings[3];
  for (int i = 0; i < 3; i++) {
    whiteReadings[i] = analogRead(i == 0 ? leftSensorPin : (i == 1 ? middleSensorPin : rightSensorPin));
    delay(100);
  }

  // find the value for the white anything less than this become the new white
  whiteThreshold = min(min(whiteReadings[0], whiteReadings[1]), whiteReadings[2]) + 50;

  Serial.print("White threshold: ");
  Serial.println(whiteThreshold);

  Serial.println("Calibration complete.");
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
