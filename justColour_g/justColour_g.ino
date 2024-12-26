//This is the entire colour code

#define s0 6    //Module pins wiring
#define s1 2
#define s2 7
#define s3 8
#define out 9

int redV;
int blueV;
int greenV;
int colour;
int data=0; 


void setup() {
pinMode(s0,OUTPUT);    
  pinMode(s1,OUTPUT);
  pinMode(s2,OUTPUT);
  pinMode(s3,OUTPUT);
  pinMode(out,INPUT);
  digitalWrite(s0,HIGH);   //Putting S0/S1 on HIGH/HIGH levels means the output frequency scalling is at 100%   (recommended)
  digitalWrite(s1,HIGH); //LOW/LOW is off HIGH/LOW is 20% and   LOW/HIGH is  2%


  Serial.begin(9600);

}
void loop() {
  colour = colourTell();
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