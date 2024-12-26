//Perfectly working manual code


#include <Servo.h>
Servo myservo;
int currentAngle = 90; // Start the servo at the midpoint, can be adjusted
int angleStep = 36;    // Define the step size for each increment/decrement

// Function declarations (prototypes)
void moveForward();
void turnRight();
void turnLeft();
void moveBackward();
void stopCar();
void opencollector();
void closecollector();
void liftcollector();

int EN1 = 10;
int EN2 = 13;
int M1A = 11;
int M1B = 12;
int M2A = 14;
int M2B = 15;
int M3A = 16;
int M3B = 17;
int M4A = 18;
int M4B = 19;

String data = "";  // Use a String to store incoming data

void setup() {
  Serial.begin(9600);    // Initialize Serial for debugging
  Serial1.begin(9600);   // Initialize Serial1 for Bluetooth communication

  // Initialize motor pins as outputs
  pinMode(EN1, OUTPUT);
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);
  pinMode(M3A, OUTPUT);
  pinMode(M3B, OUTPUT);


  // Initialize servo pins as outputs
  pinMode(19, OUTPUT);
  pinMode(M4A, OUTPUT);
  pinMode(M4B, OUTPUT);
  pinMode(8, OUTPUT);

  // Mock initialization messages
  Serial.println("Setup complete. Waiting for Bluetooth commands...");
  myservo.attach(8);
  myservo.write(currentAngle); // Initialize servo to the starting angle
}


void loop() {
  turnLeft();
  delay(50);
  turnRight();
  delay(50);
  turnRight();
  delay(50);
  turnLeft();
  delay(50);
  turnLeft();
  delay(50);
  turnRight();
  delay(50);
  turnRight();
  delay(50);
  turnLeft();
  delay(50);

  moveForward();
  delay(300);
  turnLeft();
  delay(150);
  moveForward();
  delay(500);
  turnLeft();
  delay(150);
  moveForward();
  delay(300);
  turnLeft();
  delay(100);
  moveForward();
  delay(250);
  turnLeft();
  


}

void moveForward() {
  Serial1.println("Moving forward");
  Serial.println("Moving forward");
  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
  digitalWrite(M1A, LOW);
  digitalWrite(M1B, HIGH);
  digitalWrite(M2A, HIGH);
  digitalWrite(M2B, LOW);
  digitalWrite(M3A, LOW);
  digitalWrite(M3B, LOW);
}

void turnRight() {
  Serial1.println("Turning right");
  Serial.println("Turning right");
  digitalWrite(EN1, LOW);
  digitalWrite(EN2, HIGH);
  digitalWrite(M1A, LOW);
  digitalWrite(M1B, HIGH);
  digitalWrite(M2A, HIGH);
  digitalWrite(M2B, LOW);
  digitalWrite(M3A, LOW);
  digitalWrite(M3B, LOW);
}

void turnLeft() {
  Serial1.println("Turning left");
  Serial.println("Turning left");

  digitalWrite(EN1, HIGH);
  digitalWrite(M1A, LOW);
  digitalWrite(M1B, HIGH);
  digitalWrite(EN2, LOW);
  digitalWrite(M2A, LOW);
  digitalWrite(M2B, LOW);
  digitalWrite(M3A, LOW);
  digitalWrite(M3B, LOW);
}

void moveBackward() {
  Serial1.println("Moving backward");
  Serial.println("Moving backward");
  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
  digitalWrite(M1A, HIGH);
  digitalWrite(M1B, LOW);
  digitalWrite(M2A, LOW);
  digitalWrite(M2B, HIGH);
  digitalWrite(M3A, LOW);
  digitalWrite(M3B, LOW);

}

void stopCar() {
  Serial1.println("Stopping car");
  Serial.println("Stopping car");
  digitalWrite(EN1, LOW);
  digitalWrite(M1A, LOW);
  digitalWrite(M1B, LOW);
  digitalWrite(EN2, LOW);
  digitalWrite(M2A, LOW);
  digitalWrite(M2B, LOW);
  digitalWrite(M3A, LOW);
  digitalWrite(M3B, LOW);
}

void liftcollector(){
  Serial1.println("Lifting Collector");
  Serial.println("Lifting Collector");
  digitalWrite(EN1, HIGH);
  digitalWrite(M1A, LOW);
  digitalWrite(M1B, LOW);
  digitalWrite(EN2, LOW);
  digitalWrite(M2A, LOW);
  digitalWrite(M2B, LOW);
  digitalWrite(M3A, HIGH);
  digitalWrite(M3B, LOW);
  
}

void dropcollector(){
  Serial1.println("Dropping Collector");
  Serial.println("Dropping Collector");
  digitalWrite(EN1, HIGH);
  digitalWrite(M1A, LOW);
  digitalWrite(M1B, LOW);
  digitalWrite(EN2, LOW);
  digitalWrite(M2A, LOW);
  digitalWrite(M2B, LOW);
  digitalWrite(M3A, LOW);
  digitalWrite(M3B, HIGH);
}

void opencollector() {
  String numString = data.substring(1);  // Extract substring starting from the second character
  int currentAngle = numString.toInt();
  myservo.write(currentAngle);
  Serial.print("Opening collector, current angle: ");
  Serial.println(currentAngle);

}

void closecollector() {
  String numString = data.substring(1);  // Extract substring starting from the second character
  int currentAngle = numString.toInt();
  myservo.write(currentAngle);
  Serial.print("Closing collector, current angle: ");
  Serial.println(currentAngle);
}