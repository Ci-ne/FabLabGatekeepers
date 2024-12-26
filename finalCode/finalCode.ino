#include <Wire.h>                                                                 //include the i2c library
#include <Adafruit_PWMServoDriver.h>                                              //include the PCA9685 adafruit library to easily communicate with the PCA9685 PWM module.
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();                          // called this way, it uses the default address 0x40

#define high 4096                                                                 //Value 4096 refers to 5V which is the maximum voltage that can be provided by the PCA9685 PWM module, we will use it to control the motor rotation direction
#define low 0                                                                     //Value 0 refers to 0V which is the miniimum voltage that can be provided by the PCA9685 PWM module, we will use it to control the motor rotation direction

#define in1 11                                                                     //defining const. variable named "in1" with value 0, which refers to the PCA9685 pin 0.
#define in2 12                                                                     //defining const. variable named "in2" with value 1, which refers to the PCA9685 pin 1.
#define in3 14                                                                     //defining const. variable named "in3" with value 2, which refers to the PCA9685 pin 2.
#define in4 15                                                                     //defining const. variable named "in4" with value 3, which refers to the PCA9685 pin 3.
#define enableA 10                                                                //defining const. variable named "enableA" with value 4, which refers to the PCA9685 pin 4.
#define enableB 13                                                                 //defining const. variable named "enableB" with value 5, which refers to the PCA9685 pin 5.

int const motorRight = 1;                                                         //integer constant variable named "motorRight" with value 1.
int const motorLeft = 2;                                                          //integer constant variable named "motorLeft" with value 2.

#define sensorRight 27                                                            //define a constant variable named "sensorRight" with value A0, this value refers to the PICO cennected pin.
#define sensorCenter 28                                                           //define a constant variable named "sensorCenter" with value A1, this value refers to the PICO cennected pin.
#define sensorLeft 26                                                             //define a constant variable named "sensorLeft" with value A2, this value refers to the PICO cennected pin.

int readingRight = 0;                                                             //create an integer variable to hold the right sensor reading value.
int readingCenter = 0;                                                            //create an integer variable to hold the center sensor reading value.
int readingLeft = 0;                                                              //create an integer variable to hold the left sensor reading value.

#define fullSpeed 2500                                                            //high speed of the motor. out of (4096)
#define halfSpeed 2000                                                            //low speed of the motor. out of (4096)
/*
   forward function takes two arguments.
   this function returns nothing.
   responsible for running the motor forward.
   "motorNumber" is which motor you want to run forward.
   "motorSpeed" is the speed of the motor, (fullSpeed or halfSpeed).
*/
void forward(int motorNumber, int motorSpeed) {

  if (motorNumber == 1) {                                                        //run forward the right motor.
    pwm.setPin(in3, low);
    pwm.setPin(in4, high);
    pwm.setPin(enableB, motorSpeed);
  }

  if (motorNumber == 2) {
    pwm.setPin(in1, high);                                                       //run forward the left motor.
    pwm.setPin(in2, low);
    pwm.setPin(enableA, motorSpeed);
  }
}

/*
   backward function takes two arguments.
   this function returns nothing.
   responsible for running the motor backward.
   "motorNumber" is which motor you want to run backward (motorRight or motorLeft).
   "motorSpeed" is the speed of the motor, (fullSpeed or halfSpeed).
*/
void backward(int motorNumber, int motorSpeed) {

  if (motorNumber == 1) {                                                       //run backward the right motor.
    pwm.setPin(in3, high);
    pwm.setPin(in4, low);
    pwm.setPin(enableB, motorSpeed);
  }
  if (motorNumber == 2) {
    pwm.setPin(in1, low);                                                       //run backward the left motor.
    pwm.setPin(in2, high);
    pwm.setPin(enableA, motorSpeed);
  }
}

/*
   stop function takes one argument.
   this function returns nothing.
   responsible for running the motor backward.
   "motorNumber" is which motor you want to stop running (motorRight or motorLeft).
*/

void stopMotor(int motorNumber) {
  if (motorNumber == 1) {                                                          //stop the right motor.
    pwm.setPin(in3, high);
    pwm.setPin(in4, high);
  }
  if (motorNumber == 2) {
    pwm.setPin(in1, high);                                                         //stop the left motor.
    pwm.setPin(in2, high);
  }
}

void setup() {
  pwm.begin();                                                                       //begin the i2c communication

  pinMode(sensorRight, INPUT);                                                       //set the right sensor as input pin.
  pinMode(sensorCenter, INPUT);                                                      //set the center sensor as input pin.
  pinMode(sensorLeft, INPUT);                                                        //set the left sensor as input pin.
}

void loop() {

  readingRight = digitalRead(sensorRight);                                           //read the right sensor reading and save it inside the "readingRight" variable.
  readingCenter = digitalRead(sensorCenter);                                         //read the center sensor reading and save it inside the "readingCenter" variable.
  readingLeft = digitalRead(sensorLeft);                                             //read the left sensor reading and save it inside the "readingLeft" variable.

  if (readingLeft == 0 && readingCenter == 1 && readingRight == 0) {                 //if the black line in the center of the car, run the two motors forward.
    forward(motorRight, fullSpeed);
    forward(motorLeft, fullSpeed);
  }
  if (readingLeft == 1 && readingCenter == 0 && readingRight == 0) {                 //if the black line in the left of the car, run the two motors turn left.
    forward(motorRight, halfSpeed);
    backward(motorLeft, halfSpeed);
  }
  if (readingLeft == 0 && readingCenter == 0 && readingRight == 1) {                 //if the black line in the right of the car, run the two motors turn right.
    forward(motorLeft, halfSpeed);
    backward(motorRight, halfSpeed);
  }
  if (readingLeft == 1 && readingCenter == 1 && readingRight == 1) {                 //if the three sensors read a black line at the same time, stop the two motors.
    stopMotor(motorLeft);
    stopMotor(motorRight);
  }
}
