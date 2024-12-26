#include <Servo.h>

// Pin definitions (same as before)
#define LEFT_MOTOR_FORWARD 11
#define LEFT_MOTOR_BACKWARD 12
#define RIGHT_MOTOR_FORWARD 14
#define RIGHT_MOTOR_BACKWARD 15
#define EN1  10   //RIGHT MOTOR
#define EN2  13   //LEFT MOTOR

#define IR_SENSOR_LEFT 26
#define IR_SENSOR_MIDDLE 27
#define IR_SENSOR_RIGHT 28

#define COLOR_SENSOR_S0 6
#define COLOR_SENSOR_S1 2
#define COLOR_SENSOR_S2 8
#define COLOR_SENSOR_S3 7
#define COLOR_SENSOR_OUT 9

#define CLAW_SERVO_PIN 18

#define ULTRASONIC_TRIG_PIN 20
#define ULTRASONIC_ECHO_PIN 21


// Servo for the claw
Servo clawServo;

// State definitions
enum State { LINE_FOLLOWING, OBSTACLE_DETECTION, OBSTACLE_ABSENT, OBSTACLE_DETECTED };
State currentState = LINE_FOLLOWING;

// Track junctions and turns
int junctionCount = 0;
int turnCount = 0;
int totalJunctions = 4;
int totalTurns = 6;
int redFrequency;
int blueFrequency;
int greenFrequency;

// Function prototypes
void followLine();
void handleJunction();
void handleTurn();
void detectObstacle();
void pickUpObject();
void releaseObject();
void turnLeft();
void turnRight();
bool isAtJunction();
bool isAtTurn();
long measureDistance();
String detectColor();
void  readColor();

//DC Motor definitions
int leftSpeed;
int leftMotorSpeed;
int rightSpeed;
int rightMotorSpeed;
int baseSpeed = 50;

// Constants for PID
float Kp = 1.5; // Proportional gain
float Ki = 0; // Integral gain (Start with 0, tune later)
float Kd = 0.005556; // Derivative gain
// Variables for PID
float error = 0;
float previous_error = 0;
float integral = 0;
float derivative = 0;
float correction = 0;


void setup() {
  // Initial setup (same as before)
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  
  pinMode(IR_SENSOR_LEFT, INPUT);
  pinMode(IR_SENSOR_MIDDLE, INPUT);
  pinMode(IR_SENSOR_RIGHT, INPUT);

  pinMode(COLOR_SENSOR_S0, OUTPUT);
  pinMode(COLOR_SENSOR_S1, OUTPUT);
  pinMode(COLOR_SENSOR_S2, OUTPUT);
  pinMode(COLOR_SENSOR_S3, OUTPUT);
  pinMode(COLOR_SENSOR_OUT, INPUT);
  
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  
  digitalWrite(COLOR_SENSOR_S0, HIGH);
  digitalWrite(COLOR_SENSOR_S1, LOW);
  
  clawServo.attach(CLAW_SERVO_PIN);
  clawServo.write(90); // Start with the claw open
  
  Serial.begin(9600);
}

void loop() {
  long distance = measureDistance();
  
  switch (currentState) {
    case LINE_FOLLOWING:
      followLine();
      
     /* if (isAtTurn()) {
        handleTurn();
      } else*/ if (isAtJunction()) {
        handleJunction();
      } else if (distance < 15) {
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




// Function to follow the line using IR sensors
/*void followLine() {
  int IR_SENSOR_MIDDLE = digitalRead(IR_SENSOR_LEFT);
  int IR_SENSOR_LEFT = digitalRead(IR_SENSOR_MIDDLE);
  int IR_SENSOR_RIGHT = digitalRead(IR_SENSOR_RIGHT);
  
  if (IR_SENSOR_LEFT == HIGH) {
    // Move forward
    digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
    digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
    digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
    digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  } else if (IR_SENSOR_MIDDLE == HIGH) {
    turnLeft();
  } else if (IR_SENSOR_RIGHT == HIGH) {
    turnRight();
  } else {
    // Stop the robot
    digitalWrite(LEFT_MOTOR_FORWARD, LOW);
    digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
    digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
    digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  }
}
*/
void followLine(){ 
  int LEFT_IR_VALUE = digitalRead(IR_SENSOR_LEFT);
  int RIGHT_IR_VALUE = digitalRead(IR_SENSOR_RIGHT);
  int MIDDLE_IR_VALUE = digitalRead(IR_SENSOR_MIDDLE);

  // Calculate the error
  error = calculateError(LEFT_IR_VALUE, MIDDLE_IR_VALUE, RIGHT_IR_VALUE);

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

  //leftMotorSpeed = constrain(leftSpeed, 0, 255);
  //rightMotorSpeed = constrain(rightSpeed, 0, 255);

  analogWrite(EN1, leftMotorSpeed);
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH); 
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  analogWrite(EN2, rightMotorSpeed);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH); 
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  // Set motor speeds

  // Update previous error
  previous_error = error;

  delay(10);  // Small delay for stability
}

float calculateError(int LEFT_IR_VALUE, int MIDDLE_IR_VALUE, int RIGHT_IR_VALUE) {
  // Example of a weighted sum for error calculation:
  // The line on the left side causes a negative error, on the right side causes a positive error.
  float weightedSum = -LEFT_IR_VALUE + 0*MIDDLE_IR_VALUE + RIGHT_IR_VALUE;
  float sum = LEFT_IR_VALUE + MIDDLE_IR_VALUE + RIGHT_IR_VALUE;

  // Normalize the error (avoid division by zero)
  if (sum != 0) {
    return (weightedSum / sum);
  } else {
    return 0;
  }
}

// Function to handle junctions
void handleJunction() {
  junctionCount++;
  String detectedColor = detectColor();
  if (junctionCount == 1){
  if (currentState == OBSTACLE_ABSENT) {
    turnRight();
    followLine();
  } else if (currentState == OBSTACLE_DETECTED) {
    turnLeft();
  // detect colour
    releaseObject();  // Drop the object in the warehouse
    // Turn 180 degrees
    turnRight();
    turnRight();
    currentState = OBSTACLE_ABSENT;
  }
  }
  
else if(junctionCount == 2){
    if (currentState == OBSTACLE_ABSENT) {
    turnRight();
    followLine();
  } else if (currentState == OBSTACLE_DETECTED) {
    turnLeft();
  // detect colour
    releaseObject();  // Drop the object in the warehouse
    // Turn 180 degrees
    turnLeft();
    turnLeft();
    currentState = OBSTACLE_ABSENT;

  }


else if(junctionCount == 3){
    if (currentState == OBSTACLE_ABSENT) {
    turnRight();
    followLine();
  } else if (currentState == OBSTACLE_DETECTED) {
    followLine();
  // detect colour
    releaseObject();  // Drop the object in the warehouse
    // Turn 180 degrees
    turnRight();
    turnRight();
    currentState = OBSTACLE_ABSENT;
  }
  }


else if(junctionCount == 4){
    if (currentState == OBSTACLE_ABSENT) {
    turnLeft();
    followLine();
  } else if (currentState == OBSTACLE_DETECTED) {
    followLine();
  // detect colour
    releaseObject();  // Drop the object in the warehouse
    // Turn 180 degrees
    turnRight();
    turnRight();
    currentState = OBSTACLE_ABSENT;
  }
  }


delay(1000);  // Brief delay to stabilize
}
}

// Function to handle turns
/*void handleTurn() {
  turnCount++;
  
  if (turnCount == 1 || turnCount == 3 || turnCount == 4) {
    turnLeft();
  } else if (turnCount == 2) {
    turnRight();
  }
  followLine();  // Continue line following after the turn
}
*/
// Function to stop the robot
void stopRobot() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

// Function to detect an obstacle
void detectObstacle() {
  stopRobot();
  String color = detectColor();
  pickUpObject();
  currentState = OBSTACLE_DETECTED;
}

// Function to measure distance using the ultrasonic sensor
long measureDistance() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  
  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2;
  return distance;
}

// Function to check if the robot is at a junction
bool isAtJunction() {
  switch (junctionCount){
    case 0:
      if ((digitalRead(IR_SENSOR_LEFT)==HIGH) && (digitalRead(IR_SENSOR_MIDDLE)==HIGH) && (digitalRead(IR_SENSOR_RIGHT)==HIGH)){ //condition for the junction between C and R
        return true;
        junctionCount = junctionCount +1;
      }
      else{
        return false;
      }
      break;
    case 1:
        if ((digitalRead(IR_SENSOR_MIDDLE)==HIGH) && (digitalRead(IR_SENSOR_RIGHT)==HIGH)){ //condition for the junction at R
          followLine();
          delay(500);
          stopRobot();
          if (digitalRead(IR_SENSOR_LEFT)==HIGH){
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
      if ((digitalRead(IR_SENSOR_MIDDLE)==HIGH) && (digitalRead(IR_SENSOR_LEFT)==HIGH)){ //condition for the junction between G and R
        followLine();
        delay(500);
        stopRobot();
        if (digitalRead(IR_SENSOR_MIDDLE)==HIGH){
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
      if ((digitalRead(IR_SENSOR_MIDDLE)==HIGH) && (digitalRead(IR_SENSOR_RIGHT)==HIGH)){ //condition for the junction at G
        followLine();
        delay(500);
        stopRobot();
        if (digitalRead(IR_SENSOR_MIDDLE)==HIGH){
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

// Function to check if the robot is at a turn
bool isAtTurn() {
  int LEFT_IR_VALUE = digitalRead(IR_SENSOR_LEFT);
  int RIGHT_IR_VALUE = digitalRead(IR_SENSOR_RIGHT);
  int MIDDLE_IR_VALUE = digitalRead(IR_SENSOR_MIDDLE);

  switch(turnCount){
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    if (MIDDLE_IR_VALUE == HIGH && (RIGHT_IR_VALUE ==HIGH || LEFT_IR_VALUE == HIGH )){
      turnCount = turnCount +1;
      break;
    }
    else{
      break;
    }

    case 1:
      if (MIDDLE_IR_VALUE == HIGH && (RIGHT_IR_VALUE ==HIGH || LEFT_IR_VALUE == HIGH )){
      turnCount = turnCount +1;
      break;
    }
    else{
      break;
    }

  }

  
  return (IR_SENSOR_LEFT == HIGH && (IR_SENSOR_MIDDLE == LOW || IR_SENSOR_RIGHT == LOW));
}

// Function to pick up an object
void pickUpObject() {
  clawServo.write(0);  // Close claw
  delay(1000);         // Wait for the claw to close
}

// Function to release an object
void releaseObject() {
  clawServo.write(90);  // Open claw
  delay(1000);          // Wait for the claw to open
}

// Function to turn left
void turnLeft() {  
  
  int LEFT_IR_VALUE = digitalRead(IR_SENSOR_LEFT);
  int RIGHT_IR_VALUE = digitalRead(IR_SENSOR_RIGHT);
  int MIDDLE_IR_VALUE = digitalRead(IR_SENSOR_MIDDLE);

  while ((IR_SENSOR_LEFT == HIGH) && (IR_SENSOR_MIDDLE == HIGH) && (IR_SENSOR_RIGHT == LOW)){
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);  
  }
}



// Function to turn right
void turnRight() {
  int LEFT_IR_VALUE = digitalRead(IR_SENSOR_LEFT);
  int RIGHT_IR_VALUE = digitalRead(IR_SENSOR_RIGHT);
  int MIDDLE_IR_VALUE = digitalRead(IR_SENSOR_MIDDLE);
  
  while ((IR_SENSOR_LEFT == LOW) && (IR_SENSOR_MIDDLE == HIGH) && !(IR_SENSOR_RIGHT == HIGH)){
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  }
}


void readColor() {
  // Red
  digitalWrite(COLOR_SENSOR_S2, LOW);
  digitalWrite(COLOR_SENSOR_S3, LOW);
  redFrequency = pulseIn(COLOR_SENSOR_OUT, LOW);
  
  // Green
  digitalWrite(COLOR_SENSOR_S2, HIGH);
  digitalWrite(COLOR_SENSOR_S3, HIGH);
  greenFrequency = pulseIn(COLOR_SENSOR_OUT, LOW);
  
  // Blue
  digitalWrite(COLOR_SENSOR_S2, LOW);
  digitalWrite(COLOR_SENSOR_S3, HIGH);
  blueFrequency = pulseIn(COLOR_SENSOR_OUT, LOW);
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
  return "RED";
  }
  else{
  return "NONE";
  }
}