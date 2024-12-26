//IR + Ultrasonic + Blink + Servo

#include <Servo.h>

Servo myservo;

int leftIRvalue;
int rightIRvalue;

int lwen=10;
int rwen=13;
//int fken;

int leftIR = 27;
int rightIR = 26;
int trig = 20;
int echo = 21;

long duration;
int distance;


void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  //pinMode(2, OUTPUT);
  pinMode(rightIR, INPUT);
  pinMode(leftIR, INPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(lwen, OUTPUT);
  pinMode(rwen, OUTPUT);

  Serial.begin(9600);
  myservo.attach(8);
}

// the loop function runs over and over again forever
void loop() {
  
  // rightIRvalue = digitalRead(rightIR);
  // leftIRvalue = digitalRead(leftIR);
  rightIRvalue = analogRead(rightIR);
  leftIRvalue = analogRead(leftIR);
  Serial.print("Left IR: ");
  Serial.print(leftIRvalue);
  Serial.print("    Right IR: ");
  Serial.println(rightIRvalue);
  delay(100);
}