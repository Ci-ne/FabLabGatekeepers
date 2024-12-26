#include <Servo.h>
#include <limits.h>
#include <vector>
#include <Arduino_FreeRTOS.h>

// Pin Definitions
#define s0 6
#define s1 2
#define s2 7
#define s3 8
#define out 9

#define lwen 10
#define rwen 13
#define leftMotorPin1 12
#define leftMotorPin2 11
#define rightMotorPin1 15
#define rightMotorPin2 14
#define trig 20
#define echo 21
#define sensorLeft 27
#define sensorRight 26

// Motor speed variables
int baseSpeed = 100;
int leftMotorSpeed;
int rightMotorSpeed;
int leftSpeed;
int rightSpeed;

// PID Constants
float Kp = 1.5;
float Ki = 0.0;
float Kd = 0.5;

// PID Variables
float error = 0;
float previous_error = 0;
float integral = 0;
float derivative = 0;
float correction = 0;

// Colour detection variables
int redV, blueV, greenV;
int colour;
int data = 0;

// Ultrasonic sensor variables
long duration;
int distance;
int objectDetect = 1;

Servo myservo;

// Dijkstra Variables
#define V 16
int graph[V][V] = {
    {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

// FreeRTOS tasks
void TaskLineFollowing(void *pvParameters);
void TaskColourDetection(void *pvParameters);
void TaskDijkstra(void *pvParameters);

// Function Prototypes
void LineFollower();
int colourTell();
int GetData();
float calculateError(int leftValue, int rightValue);
void setMotorSpeed(int leftSpeed, int rightSpeed);
int minDistance(int dist[], bool sptSet[]);
void dijkstra(int graph[V][V], int src, int dest);

// FreeRTOS setup
void setup() {
  Serial.begin(9600);
  myservo.attach(8);

  // Motor pin setup
  pinMode(lwen, OUTPUT);
  pinMode(rwen, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  // Colour sensor pin setup
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(out, INPUT);
  digitalWrite(s0, HIGH);
  digitalWrite(s1, HIGH);

  // Ultrasonic sensor pin setup
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  // IR sensor pin setup
  pinMode(sensorLeft, INPUT);
  pinMode(sensorRight, INPUT);

  // Create tasks
  xTaskCreate(TaskLineFollowing, "LineFollowing", 128, NULL, 2, NULL);
  xTaskCreate(TaskColourDetection, "ColourDetection", 128, NULL, 2, NULL);
  xTaskCreate(TaskDijkstra, "Dijkstra", 128, NULL, 1, NULL);

  // Start the scheduler
  vTaskStartScheduler();
}

void loop() {
  // Empty because tasks are managed by FreeRTOS
}

// Task: Line Following
void TaskLineFollowing(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    LineFollower();
    vTaskDelay(100);  // Delay for task timing
  }
}

// Task: Colour Detection
void TaskColourDetection(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    colour = colourTell();
    vTaskDelay(100);  // Delay for task timing
  }
}

// Task: Dijkstra Path Planning
void TaskDijkstra(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    dijkstra(graph, 0, 15);  // Run Dijkstra's algorithm
    vTaskDelay(5000);  // Delay to avoid flooding
  }
}

// Line following function
void LineFollower() {
  int leftValue = analogRead(sensorLeft);
  int rightValue = analogRead(sensorRight);

  error = calculateError(leftValue, rightValue);

  integral += error;
  derivative = error - previous_error;
  correction = Kp * error + Ki * integral + Kd * derivative;

  leftSpeed = baseSpeed - correction;
  rightSpeed = baseSpeed + correction;

  leftMotorSpeed = constrain(leftSpeed, 0, 255);
  rightMotorSpeed = constrain(rightSpeed, 0, 255);

  analogWrite(lwen, leftMotorSpeed);
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  analogWrite(rwen, rightMotorSpeed);
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);

  previous_error = error;

  delay(10);
}

// Colour detection function
int colourTell() {
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  redV = GetData();

  digitalWrite(s2, LOW);
  digitalWrite(s3, HIGH);
  blueV = GetData();

  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);
  greenV = GetData();

  if (redV < blueV && redV < greenV && ((blueV - redV) < (greenV - redV))) {
    colour = 1;
    Serial.println("Colour: Red");
  } else if (blueV < redV && blueV < greenV) {
    colour = 2;
    Serial.println("Colour: Blue");
  } else if (greenV < blueV && greenV < redV
