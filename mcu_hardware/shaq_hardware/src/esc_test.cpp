#include <Arduino.h>
#include <stdio.h>

#include <Servo.h>


Servo ESC1;
Servo ESC2;

int potValue;


void setup() {
  Serial.begin(115200);

  ESC1.attach(10, 1100, 1940);
  ESC2.attach(9, 1100, 1940);

  potValue = 0;
  ESC1.write(0);
  ESC2.write(0);

  ESC1.writeMicroseconds(1940);
  ESC2.writeMicroseconds(1940);
  delay(100);

  ESC1.write(0);
  ESC2.write(0);

  delay(3000);

  

  ESC1.writeMicroseconds(1940);
  ESC2.writeMicroseconds(1940);
  delay(100);

  ESC1.write(0);
  ESC2.write(0);


}

void loop() {
 

}