#include <phys253.h>          
#include <LiquidCrystal.h>
#include <Servo253.h>


#define START_SWITCH_PIN 0
#define END_SWITCH_PIN 3
#define ARM_MOTOR_OUTPUT 2

#define TRUE 1
#define FALSE 0
#define ARM_TIMEOUT 20000
#define ARM_DOWN_SPEED 600

long startArm = 0;

int digitalReadHighFilter(int pin);
void armDown();
void armUp(int motorSpeed);
void swingArm(int armSpeed);

void swingArm(int armSpeed, int kp, int kd, int threshold, int velocity, int delta) {

  
  armUp(armSpeed, kp, kd, threshold, velocity, delta);
  
  motor.stop(ARM_MOTOR_OUTPUT);

  armDown(armSpeed, kp, kd, threshold, velocity, delta);
  
  motor.stop(ARM_MOTOR_OUTPUT);
  
  tapeFollowing(kp, kd, threshold, velocity, delta);

}

void armUp(int motorSpeed, int kp, int kd, int threshold, int velocity, int delta) {

  startArm = millis();
  motor.speed(ARM_MOTOR_OUTPUT, -motorSpeed);
  delay(200);
  // LCD.clear();  
  // LCD.home();
  // LCD.setCursor(0,0); LCD.print("Swinging");
  // LCD.setCursor(0,1); LCD.print("Back");

  while(TRUE) {
    if (digitalReadHighFilter(END_SWITCH_PIN))
      break;
    if ( (millis() - startArm) >= ARM_TIMEOUT )
      break; 
    tapeFollowing(kp, kd, threshold, velocity, delta);
  }
}

void armDown(int motorSpeed, int kp, int kd, int threshold, int velocity, int delta) {
  tapeFollowing(kp, kd, threshold, velocity, delta);
  startArm = millis();
  motor.speed(ARM_MOTOR_OUTPUT, motorSpeed);
   while(TRUE){
     if (digitalReadHighFilter(START_SWITCH_PIN))
       break;
     if ( (millis() - startArm) >= ARM_TIMEOUT )
      break; 
    tapeFollowing(kp, kd, threshold, velocity, delta);
  }
}

int digitalReadHighFilter(int pin) {
  digitalWrite(pin, HIGH);
    if (digitalRead(pin) == HIGH) {
       delay(10);
       digitalWrite(pin, HIGH);
       if (digitalRead(pin) == HIGH) {
         return TRUE;
      }
    }
    return FALSE;
}
