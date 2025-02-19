#include <Arduino.h>
#include <Servo.h>

Servo ESC1;
Servo ESC2;

void setup() {
  Serial.begin(115200);


  ESC1.attach(8, 1000, 2000);
  ESC2.attach(9, 1000, 2000);

  ESC1.writeMicroseconds(1500);
  ESC2.writeMicroseconds(1500);
  delay(3000);  

  ESC1.writeMicroseconds(1500);
  ESC2.writeMicroseconds(1500);
  delay(3000);  

  ESC1.writeMicroseconds(1500);
  ESC2.writeMicroseconds(1500);
  delay(3000);



  ESC1.writeMicroseconds(1000);
  ESC2.writeMicroseconds(2000);
  delay(450);


  ESC1.writeMicroseconds(1500);
  ESC2.writeMicroseconds(1500);
  delay(100);


  ESC1.writeMicroseconds(1800); 
  ESC2.writeMicroseconds(1200);
  delay(300);


  ESC1.writeMicroseconds(1500);
  ESC2.writeMicroseconds(1500);

  
  


  
}

void loop() {
  // No continuous loop logic yet
}
