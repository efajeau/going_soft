#include <phys253.h>          
#include <LiquidCrystal.h>
#include <Servo253.h>

#define START_SWITCH_PIN 0
#define END_SWITCH_PIN 1
#define ARM_MOTOR_OUTPUT 2

void swingArm(int armSpeed) {
     LCD.clear();  
     LCD.home();
     LCD.setCursor(0,0); LCD.print("Swinging");
     LCD.setCursor(0,1); LCD.print("Back");
     
  while( digitalRead(END_SWITCH_PIN) != HIGH ){
     motor.speed(ARM_MOTOR_OUTPUT, -armSpeed);
     digitalWrite(END_SWITCH_PIN, HIGH);
  }
  
  LCD.clear();
  motor.stop(ARM_MOTOR_OUTPUT);

     LCD.home();
     LCD.setCursor(0,0); LCD.print("Swinging Arm");
     LCD.setCursor(0,1); LCD.print("Forward");
     
  while( digitalRead(START_SWITCH_PIN) != HIGH ){
     motor.speed(ARM_MOTOR_OUTPUT, armSpeed);
     digitalWrite(START_SWITCH_PIN, HIGH);
  }
  
  LCD.clear();
  
  motor.stop(ARM_MOTOR_OUTPUT);

}
