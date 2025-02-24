#include <Arduino.h>
#include <stdio.h>
#include <Servo.h>


Servo ESC1;
Servo ESC2;

void setup() {
  Serial.begin(115200);


  ESC1.attach(8, 1000, 2000);
  ESC2.attach(9, 1000, 2000);

  ESC1.writeMicroseconds(1500);
  ESC2.writeMicroseconds(1500);

  delay(8000);  

  ESC2.writeMicroseconds(2000);
  ESC1.writeMicroseconds(2000);
  delay(500);

  // ESC2.writeMicroseconds(1500);
  // delay(100);

  // ESC1.writeMicroseconds(1200); 
  // ESC2.writeMicroseconds(2000);
  // delay(750);


  // ESC1.writeMicroseconds(1500);
  // ESC2.writeMicroseconds(1500);
  // delay(100);

  // ESC1.writeMicroseconds(1800);
  // ESC2.writeMicroseconds(1200);
  // delay(500);

  
  // ESC1.writeMicroseconds(1500);
  // ESC2.writeMicroseconds(1500);

  
}

void loop() {
  // No continuous loop logic yet
}