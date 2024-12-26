//PID Line Following

#include <Servo.h>

#define s0 6    //Module pins wiring
#define s1 2
#define s2 7
#define s3 8
#define out 9

Servo myservo;

// Motor speed variables
int baseSpeed = 100;  // Base speed of the robot
int leftMotorSpeed;
int leftSpeed;
int rightMotorSpeed;
int rightSpeed;
//Motor control pins
const int leftMotorPin1 = 12; // Pin connected to left motor
const int leftMotorPin2 = 11; // Pin connected to left motor
const int rightMotorPin1 = 15; // Pin connected to right motor
const int rightMotorPin2 = 14; // Pin connected to right motor 
const int lwen=10; 
const int rwen=13;

//ultrasonic pins and variables
const int trig = 20;
const int echo = 21;
long duration;
int distance;
int objectDetect=1;

//colour detection values
int redV;
int blueV;
int greenV;
int colour;
int data=0; 

// Constants for PID control
float Kp = 1.5;  // Proportional gain
float Ki = 0.0;  // Integral gain
float Kd = 0.5;  // Derivative gain
// Variables for PID
float error = 0;
float previous_error = 0;
float integral = 0;
float derivative = 0;
float correction = 0;
// IR sensor pins
const int sensorLeft = 27;    // Left sensor connected to analog pin A0
const int sensorRight = 26;  // Right sensor connected to analog pin A2
int leftIRvalue;
int rightIRvalue;



void setup() {
  //IR
  pinMode(sensorRight, INPUT);
  pinMode(sensorLeft, INPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  //colour
  pinMode(s0,OUTPUT);    
  pinMode(s1,OUTPUT);
  pinMode(s2,OUTPUT);
  pinMode(s3,OUTPUT);
  pinMode(out,INPUT);
  digitalWrite(s0,HIGH);   //Putting S0/S1 on HIGH/HIGH levels means the output frequency scalling is at 100%   (recommended)
  digitalWrite(s1,HIGH); //LOW/LOW is off HIGH/LOW is 20% and   LOW/HIGH is  2%
  //Motor 
  pinMode(lwen, OUTPUT);
  pinMode(rwen, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  Serial.begin(9600);
  myservo.attach(8);  
}

// the loop function runs over and over again forever
void loop() {
  detectObject();
  
  if (distance<10){
    LineFollower();
  }
  else{
  colour = colourTell();
  }
}

int colourTell(){
  digitalWrite(s2,LOW);        //S2/S3   levels define which set of photodiodes we are using LOW/LOW is for RED LOW/HIGH   is for Blue and HIGH/HIGH is for green
  digitalWrite(s3,LOW);
  redV = GetData();
  
  digitalWrite(s2,LOW);
  digitalWrite(s3,HIGH);
  blueV = GetData();

  digitalWrite(s2,HIGH);
  digitalWrite(s3,HIGH);
  greenV = GetData();

  if (redV<blueV && redV<greenV && ((blueV-redV)<(greenV-redV))){
    colour=1;
    Serial.println("Colour: Red");
  }
  else if (blueV<redV && blueV<greenV){
    colour=2;
    Serial.println("Colour: blue");
  }
  else if (greenV<blueV && greenV<redV){
    colour=3;
    Serial.println("Colour: green");
  }
  else if ((greenV-redV)<(blueV-redV)){
    colour = 4;
    Serial.println("Colour: yellow");
  }
  else{
    colour=0;
    Serial.println("Colour: none of the three");
  }
  return colour;
}

int GetData(){
  data=pulseIn(out,LOW); 
  delay(20);
  return data;
}

float calculateError(int leftValue, int rightValue) {
  // Example of a weighted sum for error calculation: The line on the left side causes a negative error, on the right side causes a positive error.
  float weightedSum = -leftValue + rightValue;
  float sum = leftValue + rightValue;
  // Normalize the error (avoid division by zero)
  if (sum != 0) {
    return weightedSum / sum;
  } else {
    return 0;
  }
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Ensure the speeds are within the acceptable range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
}

void LineFollower(){ 
  int leftValue = analogRead(sensorLeft);
  int rightValue = analogRead(sensorRight);

  // Calculate the error
  error = calculateError(leftValue, rightValue);

  // PID calculations
  integral += error;
  derivative = error - previous_error;
  correction = Kp * error + Ki * integral + Kd * derivative;

  // Adjust motor speeds
  leftSpeed = baseSpeed - correction;
  Serial.println("Left Speed:");
  Serial.println(leftSpeed);
  rightSpeed = baseSpeed + correction;
  Serial.println("Right Speed:");
  Serial.println(rightSpeed);

  leftMotorSpeed = constrain(leftSpeed, 0, 255);
  rightMotorSpeed = constrain(rightSpeed, 0, 255);

  analogWrite(lwen, leftMotorSpeed);
  digitalWrite(leftMotorPin1, HIGH); 
  digitalWrite(leftMotorPin2, LOW);
  analogWrite(rwen, rightMotorSpeed);
  digitalWrite(rightMotorPin1, HIGH); 
  digitalWrite(rightMotorPin2, LOW);

  // Update previous error
  previous_error = error;

  delay(10);  // Small delay for stability
}


int detectObject(){
  //ultrasonic
  digitalWrite(trig,LOW);
  delayMicroseconds(2);
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);
  duration = pulseIn(echo,HIGH);
  distance = (duration * 0.034 / 2);  
}