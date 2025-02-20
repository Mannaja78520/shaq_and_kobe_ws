#include <Arduino.h>
<<<<<<< HEAD
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
  delay(3000);

  ESC1.writeMicroseconds(1940);
  ESC2.writeMicroseconds(1940);
  delay(5000);

  ESC1.writeMicroseconds(1940);
  ESC2.writeMicroseconds(1940);
  // delay(100);

  // ESC1.write(0);
  // ESC2.write(0);


}

void loop() {
 

}
<<<<<<< HEAD

=======
=======
#include <Servo.h>
>>>>>>> af653efcd3dd90c737cfff6178660f582c3bff7c

Servo ESC1;
Servo ESC2;

void setup() {
  Serial.begin(115200);


  ESC1.attach(8, 1000, 2000);
  ESC2.attach(9, 1000, 2000);

  ESC1.writeMicroseconds(1500);
  ESC2.writeMicroseconds(1500);
<<<<<<< HEAD
=======
<<<<<<<< HEAD:mcu_hardware/shaq_hardware/src/esc_test.cpp
>>>>>>> af653efcd3dd90c737cfff6178660f582c3bff7c
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

  
<<<<<<< HEAD

=======
========
>>>>>>> af653efcd3dd90c737cfff6178660f582c3bff7c
  delay(8000);  

  // ESC1.writeMicroseconds(1500);
  // ESC2.writeMicroseconds(1500);
  // delay(3000);  

  // ESC1.writeMicroseconds(1500);
  // ESC2.writeMicroseconds(1500);
  // delay(3000);



  ESC1.writeMicroseconds(1200); 
  ESC2.writeMicroseconds(1800);
  delay(500);


  ESC1.writeMicroseconds(1500);
  ESC2.writeMicroseconds(1500);
  delay(100);


  // ESC1.writeMicroseconds(1800); 
  // ESC2.writeMicroseconds(1200);
  // delay(450);

  
  ESC1.writeMicroseconds(1500);
  ESC2.writeMicroseconds(1500);
<<<<<<< HEAD
=======
>>>>>>>> 0b63dd0 (add dc kobe drive):mcu_hardware/shaq_hardware/test/esc_test.cpp
>>>>>>> af653efcd3dd90c737cfff6178660f582c3bff7c
  


  
}

void loop() {
  // No continuous loop logic yet
<<<<<<< HEAD
}
=======
}
>>>>>>> 0b63dd0 (add dc kobe drive)
>>>>>>> af653efcd3dd90c737cfff6178660f582c3bff7c
