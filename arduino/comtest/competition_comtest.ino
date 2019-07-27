#include <Wire.h>

byte input;

void setup() {
  Serial.begin(9600);
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Serial.println("Starting...");
}

void loop() {
    
}

void receiveEvent(int howMany) {
    char c;
  while (1 <= Wire.available()) { // loop through all but the last
    c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
  switch(c){
    case 'l':
    {
        Serial.println(" left");
        break;
    }
    case 'r':
    {
        Serial.println(" right");
        break;
    }
    case 'v':
    {
        Serial.println(" revert");
        break;
    }
    default:
        Serial.println("error");
        break;
    }
}
