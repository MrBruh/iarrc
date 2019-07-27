/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 90;    // variable to store the servo position

void setup() {
  Serial.begin(9600);
  myservo.attach(8);  // attaches the servo on pin 9 to the servo object
  pos = steer(pos, 100);
}

void loop() {
  /*for (pos = 60; pos <= 120; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 120; pos >= 60; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  */
}
//max left = 40, max right = 120, mid = 85
int steer(int current_pos, int goal){
  if(current_pos > goal){
    Serial.println("more");
    for(goal = goal; current_pos >= goal; current_pos -= 2){
      Serial.println(current_pos);
      myservo.write(current_pos);
      delay(10);
    }
  } else if(current_pos < goal) {
    Serial.println("less");
    for(goal = goal; current_pos <= goal; current_pos += 2){
      Serial.println(current_pos);
      myservo.write(current_pos);
      delay(10);
    }
  }
  return current_pos;
}
}
