#include <Servo.h>

// IR sensor definitions
const int leftIR = 26;
const int midIR = 28;
const int rightIR = 27;

// DC motor definitions
const int lmen = 10;
const int rmen = 13;
const int lm_A = 11;
const int lm_B = 12;
const int rm_A = 14;
const int rm_B = 15;
int leftSpeed, rightSpeed;
int baseSpeed = 100;

// PID constants and variables
float Kp = 74;
float Ki = 0;
float Kd = 0.2;
float error = 0, previous_error = 0, integral = 0, derivative = 0, correction = 0;

void setup() {
  // IR sensors
  pinMode(leftIR, INPUT);
  pinMode(midIR, INPUT);
  pinMode(rightIR, INPUT);
  
  // DC motors
  pinMode(lmen, OUTPUT);
  pinMode(rmen, OUTPUT);
  pinMode(lm_A, OUTPUT);
  pinMode(lm_B, OUTPUT);
  pinMode(rm_A, OUTPUT);
  pinMode(rm_B, OUTPUT);
  
  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  followLine();
}

void followLine() { 
  int leftIRValue = digitalRead(leftIR);
  int rightIRValue = digitalRead(rightIR);
  int midIRValue = digitalRead(midIR);

  Serial.print("Left IR: " + String(leftIRValue));
  Serial.print(" Mid IR: " + String(midIRValue));
  Serial.println(" Right IR: " + String(rightIRValue));


  error = calculateError(leftIRValue, midIRValue, rightIRValue);
  integral += error;
  derivative = error - previous_error;
  // correction = Kp * error + Ki * integral + Kd * derivative;
  correction = Kp * error + Kd * derivative;/

  // leftSpeed = baseSpeed - correction;
  // rightSpeed = baseSpeed + correction;

  if(correction < 0) {
    leftSpeed = baseSpeed - correction;
    leftSpeed = constrain(leftSpeed, 0, 255);
    moveRightMotor(leftSpeed);
    delay(70);
    stopMotors();
    delay(10);
  }
  else if(correction > 0) {
    rightSpeed = baseSpeed + correction;
    rightSpeed = constrain(rightSpeed, 0, 255);
    moveLeftMotor(rightSpeed);
    delay(40);
    stopMotors();
    delay(10);
  }
  else if(correction == 0 && leftIRValue == 0 && rightIRValue == 0 && midIRValue == 0) moveRightMotor(200);
  else moveLeftMotor(200); moveRightMotor(200); delay(90); stopMotors(); delay(200);
  Serial.println("correction: " + String(correction));
 
 

  // moveMotors(leftSpeed, rightSpeed);
 

  // Serial.println("Left Speed: " + String(leftSpeed));
  // Serial.println("Right Speed: " + String(rightSpeed));

  previous_error = error;
}

void moveMotors(int leftSpeed, int rightSpeed) {
  analogWrite(lmen, leftSpeed);
  digitalWrite(lm_A, HIGH); 
  digitalWrite(lm_B, LOW);
  analogWrite(rmen, rightSpeed);
  digitalWrite(rm_A, HIGH); 
  digitalWrite(rm_B, LOW);
}
void moveLeftMotor(int speed) {
  analogWrite(lmen, speed+100);
  digitalWrite(lm_A, HIGH); 
  digitalWrite(lm_B, LOW);
}
void moveRightMotor(int speed) {
  analogWrite(rmen, speed-50);
  digitalWrite(rm_A, HIGH); 
  digitalWrite(rm_B, LOW);
}

void stopMotors(){
  analogWrite(lmen, 225);
  analogWrite(rmen, 225);
  digitalWrite(lm_A, LOW); 
  digitalWrite(lm_B, LOW);
  digitalWrite(rm_A, LOW); 
  digitalWrite(rm_B, LOW);
}

float calculateError(int leftIRValue, int midIRValue, int rightIRValue) {
  float weightedSum = (-leftIRValue) + (0 * midIRValue) + (rightIRValue);
  float sum = leftIRValue + midIRValue + rightIRValue;
  return (sum != 0) ? (weightedSum / sum) : 0;
}