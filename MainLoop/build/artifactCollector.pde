#include <phys253.h>          
#include <LiquidCrystal.h>
#include <Servo253.h>


#define START_SWITCH_PIN 0
#define END_SWITCH_PIN 3
#define ARM_MOTOR_OUTPUT 2

#define TRUE 1
#define FALSE 0

int digitalReadHighFilter(int pin);

void swingArm(int armSpeed) {
  LCD.clear();  
  LCD.home();
  LCD.setCursor(0,0); LCD.print("Swinging");
  LCD.setCursor(0,1); LCD.print("Back");
  

  while(TRUE){
    if (digitalReadHighFilter(END_SWITCH_PIN)) {
      break;
    }
    motor.speed(ARM_MOTOR_OUTPUT, -armSpeed);
  }
  
  LCD.clear();
  motor.stop(ARM_MOTOR_OUTPUT);

  LCD.home();
  LCD.setCursor(0,0); LCD.print("Swinging Arm");
  LCD.setCursor(0,1); LCD.print("Forward");
 
  while(TRUE){
     if (digitalReadHighFilter(START_SWITCH_PIN)) {
       break;
      }
    motor.speed(ARM_MOTOR_OUTPUT, 300);
  }
  
  LCD.clear();
  
  motor.stop(ARM_MOTOR_OUTPUT);
  setLastError();
}

int digitalReadHighFilter(int pin) {
  digitalWrite(pin, HIGH);
    if (digitalRead(pin) == HIGH) {
       delay(50);
       digitalWrite(pin, HIGH);
       if (digitalRead(pin) == HIGH) {
         return TRUE;
      }
    }
    return FALSE;
}
