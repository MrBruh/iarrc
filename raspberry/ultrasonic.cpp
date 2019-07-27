#include <Wire.h>

//{trig, echo}
//Centre Var
//int CPins[] = {6, 7};
//Left Close Var
int LCPins[] = {13, 12};
int LC_distance = 0;
//Left Far Var
int LFPins[] = {10, 9};
int LF_distance = 0;
//Right Close Var
int RCPins[] = {7, 6};
int RC_distance = 0;
//Right Far Var
int RFPins[] = {4, 3};
int RF_distance = 0;

//communication vars
int delay_millis = 500;

void setup() {
  Serial.begin(9600);
  Wire.begin(); // join i2c bus (address optional for master)
  /*pinMode(CPins[0], OUTPUT);
  pinMode(CPins[1], INPUT); */
  pinMode(LCPins[0], OUTPUT);
  pinMode(LCPins[1], INPUT);
  pinMode(LFPins[0], OUTPUT);
  pinMode(LFPins[1], INPUT);
  pinMode(RCPins[0], OUTPUT);
  pinMode(RCPins[1], INPUT);
  pinMode(RFPins[0], OUTPUT);
  pinMode(RFPins[1], INPUT);
}

void loop() {
  //Serial.print("LF LC RC RF: ");
  LF_distance = check_ultrasonic(LFPins);
  LC_distance = check_ultrasonic(LCPins);
  RC_distance = check_ultrasonic(RCPins);
  RF_distance = check_ultrasonic(RFPins);
//Centre Reading
  /*
  Serial.print("C D = ");
  if (check_ultrasonic(CPins) <= 25.0){
      Serial.print(" Center ");
  }
  */
//Left Close & Far Reading  
 
  if (LC_distance <= 25.0 || LF_distance <= 25.0){
      Serial.println("l");
      Wire.beginTransmission(8); 
      Wire.write("l");                      
      Wire.endTransmission();    
  } 

//Right Close & Far Reading
  
  
  if (RC_distance <= 25.0 || RF_distance <= 25.0){
     Serial.println("r");
     Wire.beginTransmission(8); 
      Wire.write("r");                      
      Wire.endTransmission();
  } else if (LC_distance > 25.0 && LF_distance > 25.0){
    Serial.println("v");
    Wire.beginTransmission(8); 
      Wire.write("v");                      
      Wire.endTransmission();
  } else {
    Serial.println(" ");
  }
  delay(1000);
}

float check_ultrasonic(int pinArray[]){
  float distance, duration = 0.0;

  digitalWrite(pinArray[0], LOW); 
  delayMicroseconds(2); 
  digitalWrite(pinArray[0], HIGH);
  delayMicroseconds(10);
  digitalWrite(pinArray[0], LOW);
  
  duration = pulseIn(pinArray[1], HIGH);
  distance = (duration / 2) * 0.0344;
  
  /*Serial.print(distance);
  Serial.print(" cm ");*/
  return distance;
}