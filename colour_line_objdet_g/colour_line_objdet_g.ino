//colour + object detection + line following

#include <Servo.h>

//colour definitions
#define s0 6    //Module pins wiring
#define s1 2
#define s2 8
#define s3 7
#define out 9
int data=0; 
int redV;
int blueV;
int greenV;
int colour;

//IR definitions
int leftIRValue;
int rightIRValue;
int midIRValue;
const int leftIR = 26;
const int midIR = 27;
const int rightIR = 28;

//Ultrasonic
const int trig = 21;
const int echo = 20;
long duration;
int distance;
int dis;
int col;

//Servo motor definitions
Servo myservo;

//DC motor defintions
const int lmen=10; 
const int rmen=13;
const int lm_A = 11;
const int lm_B = 12;
const int rm_A = 14;
const int rm_B = 15;
const int fk_A = 16;
const int fk_B = 17;
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
  //IR pinMode
  pinMode(rightIR, INPUT);
  pinMode(leftIR, INPUT);
  pinMode(midIR, INPUT);
  //Ultrasonic pinMode
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  //Colour pinMode
  pinMode(s0,OUTPUT);    //pin modes
  pinMode(s1,OUTPUT);
  pinMode(s2,OUTPUT);
  pinMode(s3,OUTPUT);
  pinMode(out,INPUT);
  //DC motors pinMode
  pinMode(lmen, OUTPUT);
  pinMode(rmen, OUTPUT);
  pinMode(lm_A,OUTPUT);
  pinMode(lm_B,OUTPUT);
  pinMode(rm_A,OUTPUT);
  pinMode(rm_B,OUTPUT);
  pinMode(fk_A,OUTPUT);
  pinMode(fk_B,OUTPUT);

  
  //frequency of colour sensor
  digitalWrite(s0,HIGH);   //Putting S0/S1 on HIGH/HIGH levels means the output frequency scalling is at 100%   (recommended)
  digitalWrite(s1,HIGH); //LOW/LOW is off HIGH/LOW is 20% and   LOW/HIGH is  2%

  //Servo motor definition
  myservo.attach(8);
  //Serial monitor definition
  Serial.begin(9600);
}

// the loop function runs over and over again forever
void loop() {
 /* while(detectObject()>4){
    followLine();
  }
  stopMoving();
  dis = detectObject();
  Serial.print("object at: ");
  Serial.println(dis);
  col = colourTell();
  Serial.print("Colour: ");
  Serial.println(col);
  */
  

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


void stopMoving(){
  analogWrite(rmen, 0);
  analogWrite(lmen, 0);
  digitalWrite(lm_A, LOW);
  digitalWrite(lm_B,LOW);
  digitalWrite(rm_A, LOW);
  digitalWrite(rm_B,LOW);
  Serial.println("Stop moving");
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
  Serial.print("distance: ");
  Serial.println(distance);
  return distance;
}

void followLine(){ 
  int leftIRValue = analogRead(leftIR);
  int rightIRValue = analogRead(rightIR);
  int midIRValue = analogRead(midIR);

  // Calculate the error
  error = calculateError(leftIRValue, midIRValue, rightIRValue);

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

  analogWrite(lmen, leftMotorSpeed);
  digitalWrite(lm_A, HIGH); 
  digitalWrite(lm_B, LOW);
  analogWrite(rmen, rightMotorSpeed);
  digitalWrite(rm_A, HIGH); 
  digitalWrite(rm_B, LOW);
  // Set motor speeds

  // Update previous error
  previous_error = error;

  delay(10);  // Small delay for stability
}

float calculateError(int leftIRValue, int midIRvalue, int rightIRValue) {
  // Example of a weighted sum for error calculation:
  // The line on the left side causes a negative error, on the right side causes a positive error.
  float weightedSum = -leftIRValue + 0*midIRValue + rightIRValue;
  float sum = leftIRValue + midIRValue + rightIRValue;

  // Normalize the error (avoid division by zero)
  if (sum != 0) {
    return (weightedSum / sum);
  } else {
    return 0;
  }
}


/*void moveForward(){
  analogWrite(rmen, 255);
  analogWrite(lmen, 255);
  digitalWrite(lm_A, HIGH);
  digitalWrite(lm_B,LOW);
  digitalWrite(rm_A, HIGH);
  digitalWrite(rm_B,LOW);
  Serial.println("Moving forward");
}

void turnLeft(){ //TRIED IMPLEMENTING SPIN INSTEAD OF NORMAL TURN HERE
  analogWrite(rmen, 255);
  analogWrite(lmen, 0);
  digitalWrite(rm_B, LOW);
  digitalWrite(rm_A,HIGH);
  digitalWrite(lm_B, HIGH);
  digitalWrite(lm_A,LOW);
  Serial.println("turning Left");
}

void turnRight(){
  analogWrite(lmen, 255);
  analogWrite(rmen, 0);
  digitalWrite(lm_B, LOW);
  digitalWrite(lm_A,HIGH);  
  Serial.println("turning Right");
}*/