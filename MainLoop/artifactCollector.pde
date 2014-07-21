#include <phys253.h>          
#include <LiquidCrystal.h>
#include <Servo253.h>

#define START_SWITCH_PIN 0
#define END_SWITCH_PIN 3
#define ARM_MOTOR_OUTPUT 2

void swingArm(int armSpeed) {
  LCD.clear();  
  LCD.home();
  LCD.setCursor(0,0); LCD.print("Swinging");
  LCD.setCursor(0,1); LCD.print("Back");
  

  while(TRUE){
    digitalWrite(END_SWITCH_PIN, HIGH);
    if (digitalRead(END_SWITCH_PIN) == HIGH) {
      delay(50);
      digitalWrite(END_SWITCH_PIN, HIGH);
      if (digitalRead(END_SWITCH_PIN) == HIGH) {
        break;
      }
    }
    motor.speed(ARM_MOTOR_OUTPUT, -armSpeed);

  }
  
  LCD.clear();
  motor.stop(ARM_MOTOR_OUTPUT);

  LCD.home();
  LCD.setCursor(0,0); LCD.print("Swinging Arm");
  LCD.setCursor(0,1); LCD.print("Forward");
 
  while(TRUE){
    digitalWrite(START_SWITCH_PIN, HIGH);
    if (digitalRead(START_SWITCH_PIN) == HIGH) {
       delay(50);
       digitalWrite(START_SWITCH_PIN, HIGH);
       if (digitalRead(START_SWITCH_PIN) == HIGH) {
       break;
      }
    }
    motor.speed(ARM_MOTOR_OUTPUT, 300);

  }
  
  LCD.clear();
  
  motor.stop(ARM_MOTOR_OUTPUT);

}
