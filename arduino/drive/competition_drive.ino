#include <Servo.h>      //modified servo library
#include <MINDSi.h>     //Minds-i library

Servo drive, steer;     //name ESC and steering servo

int dtwait = 75;

void setup(){
  Serial.begin(9600);   //begin serial session
  drive.attach(5);      //assign pin 5 to ESC
  steer.attach(4);      //assign pin 4 to steering servo
  drive.write(87);      //send neutral signal to ESC
  steer.write(90);      //send neutral signal to steering servo
  delay(2000);          //to arm ESC
}

void loop(){
  if(getPing(8) < 1800){
    fwdslowStop();
    brake();
    steer.write(160);
    reverse();     
  }
  if(getPing(9) < 1800){
    fwdslowStop();
    brake();
    steer.write(35);
    reverse();     
  }
  if(getPing(10) < 1800){
    fwdslowStop();
    brake();
    steer.write(35);
    reverse();     
  }
  else{
    forward();
  }
  if(getPing(11) < 1800){
    revslowStop();
    forward();
    }      
  //testSig();
}

void fwdslowStop(){  
  int pos1 = 87;  //this for loop slows the wheels back to stop
  for (pos1 = 100; pos1 >= 87; pos1 -= 1) {
    drive.write(pos1);
    delay(dtwait);      
  }
}

void revslowStop(){
  int pos2 = 70;
  for (pos2 = 70; pos2 < 87; pos2 += 1) {
    drive.write(pos2);    //for loop to ramp wheels back down to stop
    delay(dtwait);
  }
}

void forward(){
  steer.write(90);
  int pos1 = 87;    //for loop to ramp wheels up to forward speed
   for (pos1 = 87; pos1 < 100; pos1 += 1){
    drive.write(pos1);
   }
}

void reverse(){ 
  int pos2 = 87;    //for loop to ramp up wheels in reverse
   for (pos2 = 87; pos2 >= 70; pos2 -= 1){
    drive.write(pos2);
    delay(dtwait);
   }
    for (pos2 = 70; pos2 < 87; pos2 += 1) {
    drive.write(pos2);    //for loop to ramp wheels back down to stop
    delay(dtwait);
  }  
}

void steerRight(){
  steer.write(35);   //turn wheels to right.
}

void steerLeft(){
  steer.write(160);  //turn wheels to the left.
}

void brake(){
  drive.write(20);      //a short reverse signal
  delay(200);           //delay for ESC to respond and reset
  drive.write(90);      //send neutral signal to ESC
  delay(200);           //delay for ESC to re-arm
}

void testSig() {
  Serial.print(getPing(8)/29/2);
  Serial.print(",");
  Serial.print(getPing(9)/29/2);
  Serial.print(",");
  Serial.print(getPing(10)/29/2);
  Serial.print(",");
  Serial.print(getPing(11)/29/2);
  Serial.print(",");
  Serial.print("*");
}

void passLeft(){
  steer.write(35);   //turn wheels to right.
}

void passRight(){
  steer.write(160);  //turn wheels to the left.
}
