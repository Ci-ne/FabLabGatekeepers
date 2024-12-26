const int trig = 21;
const int echo = 20;
long duration;
int distance;
int dist;


void setup() {
  // put your setup code here, to run once:
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  dist = detectObject();

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
  Serial.println(distance);
  return distance; 
}