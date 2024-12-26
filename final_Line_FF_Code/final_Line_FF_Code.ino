#include <Wire.h>
#include <Servo.h>
#include <NewPing.h>

Servo clawServo;  //Value 0 refers to 0V which is the miniimum voltage that can be provided by the PCA9685 PWM module, we will use it to control the motor rotation direction

//Motor
#define mLeftA 11     //defining const. variable named "in1" with value 0, which refers to the PCA9685 pin 0.
#define mLeftB 12   //defining const. variable named "in2" with value 1, which refers to the PCA9685 pin 1.
#define mRightA 14    //defining const. variable named "in3" with value 2, which refers to the PCA9685 pin 2.
#define mRightB 15  //defining const. variable named "in4" with value 3, which refers to the PCA9685 pin 3.
#define enableLeft 10                                                                //defining const. variable named "enableA" with value 4, which refers to the PCA9685 pin 4.
#define enableRight 13                                                                 //defining const. variable named "enableB" with value 5, which refers to the PCA9685 pin 5.
#define mUpA 16
#define mUpB 17


//int const motorRight = 1;                                                         //integer constant variable named "motorRight" with value 1.
//int const motorLeft = 2;                                                          //integer constant variable named "motorLeft" with value 2.

//IR Sensor
#define sensorRight 27   //define a constant variable named "sensorRight" with value A0, this value refers to the PICO cennected pin.
#define sensorCenter 28  //define a constant variable named "sensorCenter" with value A1, this value refers to the PICO cennected pin.
#define sensorLeft 26    //define a constant variable named "sensorLeft" with value A2, this value refers to the PICO cennected pin.
int IRsensorLeft = 1;
int IRsensorCenter = 1;
int IRsensorRight = 1;

//Color Sensor
#define S0 6
#define S1 3
#define S2 8
#define S3 7
#define sensorOut 9
int color=0;
int frequency = 0;

// Claw Servo
#define CLAW_SERVO_PIN 18

//Ultrasonic
#define ULTRASONIC_TRIG_PIN 20
#define ULTRASONIC_ECHO_PIN 21
#define MAX_DISTANCE 50

bool ballInHand=false;

NewPing sonar(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN, MAX_DISTANCE); //make sure to install this library

int error, last_error, MV, pid_l, pid_r, Df, Df1, Df2, Df3, I, I1, I2, I3, P, Pd, bitSensor;
int Max_MV;
unsigned char Kp = 40;
unsigned char Kd = 0;
unsigned char Ts = 1;
unsigned char maxPwm = 180;

// State definitions
enum State { LINE_FOLLOWING, OBSTACLE_DETECTION, OBSTACLE_ABSENT, OBSTACLE_DETECTED };
State currentState = LINE_FOLLOWING;
unsigned char intersection = 0;

// Track junctions and turns
int junctionCount = 0;
int turnCount = 0;
int totalJunctions = 4;
int totalTurns = 6;
int redFrequency;
int blueFrequency;
int greenFrequency;

void setup() {

  //Motor L298N
  pinMode(mRightA, OUTPUT);
  pinMode(mRightB, OUTPUT);
  pinMode(mLeftA, OUTPUT);
  pinMode(mLeftB, OUTPUT);
  pinMode(enableLeft, OUTPUT);
  pinMode(enableRight, OUTPUT);
  //IR Sensor
  pinMode(sensorRight, INPUT);   //set the right sensor as input pin.
  pinMode(sensorCenter, INPUT);  //set the center sensor as input pin.
  pinMode(sensorLeft, INPUT);    //set the left sensor as input pin.
  //Color Sensor
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  // Set Pulse Width scaling to 100%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH);

  //boxServo.attach(11); 
  //declaring ultrasonic pins
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  clawServo.attach(CLAW_SERVO_PIN);
  clawServo.write(90); // Start with the claw open

  Serial.begin(9600);  //begin the i2c communication

}

void loop() {
    long distance = measureDistance();

    switch (currentState) {
    case LINE_FOLLOWING:
      lineFollowing();
      
      if (isAtJunction()) {
        handleJunction();
      } else if (distance < 15 && !ballInHand) {
        currentState = OBSTACLE_DETECTION;
      }
      break;
      
    case OBSTACLE_DETECTION:
      detectObstacle();
      break;
      
    case OBSTACLE_ABSENT:
      currentState = LINE_FOLLOWING;
      break;
      
    case OBSTACLE_DETECTED:
      handleJunction(); // Follow the necessary actions at junctions
      break;
  }
}

void(* resetFunc) (void) = 0;

void readSensor() {
  IRsensorLeft = digitalRead(sensorLeft);      //read the left sensor reading
  IRsensorCenter = digitalRead(sensorCenter);  //read the center sensor reading
  IRsensorRight = digitalRead(sensorRight);    //read the right sensor reading
}

void lineFollowing() {
  readSensor();
  String bitSensor = (String(IRsensorLeft)+ String(IRsensorCenter)+ String(IRsensorRight));

  if (bitSensor == "100") {
    error = -2;
  } 
  // else if (bitSensor == "110") {
  //     error = -1;
  // } 
  else if (bitSensor == "010") {
      error = 0;
  } 
  // else if (bitSensor == "011") {
  //     error = 1;
  // } 
  else if (bitSensor == "001") {
      error = 2;
  }

    Max_MV = Kp * 2;
    P = Kp * error;
    Df1 = Kd * 8;
    Df2 = Df1 / Ts;
    Df3 = error - last_error;
    Df = Df2 * Df3;

    last_error = error;
    MV = P + Df;
    //Serial.println(MV);
    if (MV >= -Max_MV && MV <= Max_MV) {
      pid_l = maxPwm + MV;
      pid_r = maxPwm - MV;
      if (pid_l < 0) pid_l = 0;
      if (pid_l > 255) pid_l = 255;
      if (pid_r < 0) pid_r = 0;
      if (pid_r > 255) pid_r = 255;
      forward(pid_l, pid_r);
    } else if (MV < -Max_MV) {
      turnLeft(150, 90);
    } else if (MV > Max_MV) {
      turnRight(90, 150);
    } else {
      pid_r, pid_l = maxPwm;
      forward(pid_r, pid_l);
    }
}

void readColor() {
  // Red
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  redFrequency = pulseIn(sensorOut, LOW);
  
  // Green
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  greenFrequency = pulseIn(sensorOut, LOW);
  
  // Blue
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blueFrequency = pulseIn(sensorOut, LOW);
}


// Function to detect color based on sensor readings
String detectColor() {
  readColor();
  if (redFrequency<blueFrequency && redFrequency<greenFrequency && ((blueFrequency-redFrequency)<(greenFrequency-redFrequency))){
 return "RED";
  }
  else if (blueFrequency<redFrequency && blueFrequency<greenFrequency){
    return "BLUE";
  }
  else if (greenFrequency<blueFrequency && greenFrequency<redFrequency){
  return "GREEN";
  }
  else if ((greenFrequency-redFrequency)<(blueFrequency-redFrequency)){
  return "YELLOW";
  }
  else{
  return "NONE";
  }
}

void forward(int valLeft, int valRight){
    digitalWrite(mLeftA, LOW);
    digitalWrite(mLeftB, HIGH);
    analogWrite(enableRight, valRight);
    digitalWrite(mRightA, LOW);
    digitalWrite(mRightB, HIGH);
    analogWrite(enableLeft, valLeft);
    Serial.println("Forward: " + String(valLeft) + ", " + String(valRight));
}

void turnRight(int valLeft, int valRight){
    digitalWrite(mLeftA, HIGH);
    digitalWrite(mLeftB, LOW);
    analogWrite(enableRight, valRight);
    digitalWrite(mRightA, LOW);
    digitalWrite(mRightB, HIGH);
    analogWrite(enableLeft, valLeft);
    //Serial.println("Right: " + String(valLeft) + ", " + String(valRight));
}


void turnLeft(int valLeft, int valRight){
    digitalWrite(mLeftA, LOW);
    digitalWrite(mLeftB, HIGH);
    analogWrite(enableRight, valRight);
    digitalWrite(mRightA, HIGH);
    digitalWrite(mRightB, LOW);
    analogWrite(enableLeft, valLeft);
    //Serial.println("Left: " + String(valLeft) + ", " + String(valRight));
}

void stopRobot(){
    digitalWrite(mLeftA, LOW);
    digitalWrite(mLeftB, LOW);
    digitalWrite(mRightA, LOW);
    digitalWrite(mRightB, LOW);
}


long measureDistance() {
    delay(50);
   unsigned int distance = sonar.ping_cm();
   Serial.print(distance);
   Serial.println("cm");
  return distance;
}

// Function to detect an obstacle
void detectObstacle() {
  stopRobot();
  String color = detectColor();
  //pickUpObject();
  currentState = OBSTACLE_DETECTED;
}

// Function to pick up an object
/*void pickUpObject() {
  /*forward(150,150);
  clawServo.write(0);  // Close claw
  ballInHand = true;
  delay(1000);   */      // Wait for the claw to close
 /* Serial1.println("Lifting Collector");
  Serial.println("Lifting Collector");
  digitalWrite(enableLeft, HIGH);
  digitalWrite(mLeftA, LOW);
  digitalWrite(mLeftB, LOW);
  digitalWrite(enableRight, LOW);
  digitalWrite(mRightA, LOW);
  digitalWrite(mRightB, LOW);
  digitalWrite(mUpA, HIGH);
  digitalWrite(mUpB, LOW);
  */




// Function to release an object
/*void releaseObject() {
  /*clawServo.write(90);  // Open claw
  ballInHand = false;
  delay(1000);     */     // Wait for the claw to open
  /*Serial1.println("Dropping Collector");
  Serial.println("Dropping Collector");
  digitalWrite(enableLeft, HIGH);
  digitalWrite(mLeftA, LOW);
  digitalWrite(mLeftB, LOW);
  digitalWrite(enableRight, LOW);
  digitalWrite(mRightA, LOW);
  digitalWrite(mRightB, LOW);
  digitalWrite(mUpA, LOW);
  digitalWrite(mUpB, HIGH);
}*/

bool isAtJunction(){
  readSensor();
  String bitSensor = (String(IRsensorLeft)+ String(IRsensorCenter)+ String(IRsensorRight));

    switch (junctionCount){
    case 0:
      if (bitSensor == "111"){ //condition for the junction between C and R
        return true;
        junctionCount = junctionCount +1;
      }
      else{
        return false;
      }
      break;
    case 1:
        if (bitSensor = "011"){ //condition for the junction at R
          lineFollowing();
          delay(500);
          stopRobot();
          if (digitalRead(IRsensorLeft)==HIGH){
            return true;
            junctionCount = junctionCount +1;
          }
          else{
          return false;
          }
        }
        else{
          return false;
        }
        break;
    case 2:
      if (bitSensor=="110"){ //condition for the junction between G and R
        lineFollowing();
        delay(500);
        stopRobot();
        if (digitalRead(IRsensorCenter)==HIGH){
          return true;
          junctionCount = junctionCount +1;
        }
        else{
          return false;
        }
      }
      else{
          return false;
        }
      break;
    case 3:
      if (bitSensor=="011"){ //condition for the junction at G
        lineFollowing();
        delay(500);
        stopRobot();
        if (digitalRead(IRsensorCenter)==HIGH){
          return true;
          junctionCount = junctionCount +1;
        }
        else{
          return false;
        }
      }
      else{
          return false;
        }
      break;
    default:
        return false;      
}

}

void handleJunction() {
  junctionCount++;
  String detectedColor = detectColor();
  if (junctionCount == 1){
  if (!ballInHand) {
    turnRight(90,150);
    lineFollowing();
  } else if (currentState == OBSTACLE_DETECTED && ballInHand) {
    if(detectedColor == "YELLOW"){
      turnLeft(150,90);
      readSensor();
      String bitSensor = (String(IRsensorLeft)+ String(IRsensorCenter)+ String(IRsensorRight));
      while(bitSensor!="111"){
        lineFollowing();
      }
      stopRobot();
    
      // Turn 180 degrees
      turnRight(90,150);
      turnRight(90,150);
      currentState = OBSTACLE_ABSENT;
      detectedColor=="NONE";
    }
    else if (detectedColor == "GREEN"){
      turnRight(90,150);
      readSensor();
      String bitSensor = (String(IRsensorLeft)+ String(IRsensorCenter)+ String(IRsensorRight));
      while(bitSensor!="111"){
        lineFollowing();
      }
      stopRobot();
   
      // Turn 180 degrees
      turnRight(90,150);
      turnRight(90,150);
      currentState = OBSTACLE_ABSENT;
      detectedColor=="NONE";
    }
  }
  }
else if(junctionCount == 2){
    if (currentState == OBSTACLE_ABSENT) {
    turnRight(90,150);
    lineFollowing();
  } else if (currentState == OBSTACLE_DETECTED && ballInHand) {
    turnLeft(150,90);
  // detect colour
   
    // Turn 180 degrees
    turnLeft(150,90);
    turnLeft(150,90);
    currentState = OBSTACLE_ABSENT;

  }
}


else if(junctionCount == 3){
    if (currentState == OBSTACLE_ABSENT) {
    turnRight(90,150);
    lineFollowing();
  } else if (currentState == OBSTACLE_DETECTED && ballInHand) {
    if (detectedColor=="RED"){
    readSensor();
      String bitSensor = (String(IRsensorLeft)+ String(IRsensorCenter)+ String(IRsensorRight));
      while(bitSensor!="111"){
        lineFollowing();
      }
      stopRobot();
   
    turnRight(90,150);
    turnRight(90,150);
    
    }
    
    currentState = OBSTACLE_ABSENT;
  }
}
  


else if(junctionCount == 4){
    if (currentState == OBSTACLE_ABSENT) {
    turnLeft(150,90);
    readSensor();
      String bitSensor = (String(IRsensorLeft)+ String(IRsensorCenter)+ String(IRsensorRight));
      while(bitSensor!="111"){
        lineFollowing();
      }

  } else if (currentState == OBSTACLE_DETECTED && ballInHand) {
    lineFollowing();
  // detect colour
   
    // Turn 180 degrees
    turnRight(45,90);
    turnRight(45,90);
    currentState = OBSTACLE_ABSENT;
  }
  }
}






