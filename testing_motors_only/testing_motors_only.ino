const int lmen=10; 
const int rmen=13;
const int lm_A = 11;
const int lm_B = 12;
const int rm_A = 14;
const int rm_B = 15;

void setup() {
  // put your setup code here, to run once:
  pinMode(lmen, OUTPUT);
  pinMode(rmen, OUTPUT);
  pinMode(lm_A,OUTPUT);
  pinMode(lm_B,OUTPUT);
  pinMode(rm_A,OUTPUT);
  pinMode(rm_B,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(lmen, 255);
  digitalWrite(lm_A, HIGH); 
  digitalWrite(lm_B, LOW);
  analogWrite(rmen, 255);
  digitalWrite(rm_A, HIGH); 
  digitalWrite(rm_B, LOW);
  delay(500);
  analogWrite(lmen, 0);
  digitalWrite(lm_A, LOW); 
  digitalWrite(lm_B, LOW);
  analogWrite(rmen, 0);
  digitalWrite(rm_A, LOW); 
  digitalWrite(rm_B, LOW);
  delay(500);

  // Set motor speeds
}
