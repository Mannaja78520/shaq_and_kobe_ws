#include <Arduino.h>

#define pina 7
#define pinb 6
#define pwm 2

void setup(){
    pinMode(pina,OUTPUT);
    pinMode(pinb,OUTPUT);
    pinMode(pwm,OUTPUT);
    digitalWrite(pina,HIGH);
    digitalWrite(pinb,LOW);
    
    analogWrite(pwm,255);


}

void loop(){

    analogWrite(pwm,100);
    delay(3000);
    analogWrite(pwm,200);
    delay(3000);
    digitalWrite(pina,HIGH);
    digitalWrite(pinb,LOW);
    analogWrite(pwm,100);
    delay(3000);
    analogWrite(pwm,200);


}