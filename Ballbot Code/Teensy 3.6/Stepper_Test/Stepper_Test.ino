int PUL=7; //define Pulse pin
int DIR=8; //define Direction pin
int ENA=5; //define Enable Pin
void setup() {
//  Serial.begin(38400);     // opens serial port, sets data rate to 9600 bps
  pinMode (PUL, OUTPUT);
  pinMode (DIR, OUTPUT);
  pinMode (ENA, OUTPUT);
  digitalWrite(ENA,LOW);
}

void loop() {
  Serial.println("forward"); 
  for (int i=0; i<6400; i++)    //Forward 5000 steps
  {
    digitalWrite(DIR,LOW);
    digitalWrite(PUL,HIGH);
    delayMicroseconds(1000);
    digitalWrite(PUL,LOW);
    delayMicroseconds(1000);
  }   
  
  Serial.println("backwards"); 
  for (int i=0; i<6400; i++)   //Backward 5000 steps
  {
    digitalWrite(DIR,HIGH);
    digitalWrite(ENA,LOW);
    digitalWrite(PUL,HIGH);
    delayMicroseconds(50);
    digitalWrite(PUL,LOW);
    delayMicroseconds(50);
  }
}
