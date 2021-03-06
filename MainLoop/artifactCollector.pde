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
  
  tapeFollowing(kp, kd, threshold, velocity, delta);
  
  armUp(armSpeed, kp, kd, threshold, velocity, delta);

  tapeFollowing(kp, kd, threshold, velocity, delta);
  
  armDown(armSpeed, kp, kd, threshold, velocity, delta);
  
  tapeFollowing(kp, kd, threshold, velocity, delta);
  
  

}

void swingArmTurning(int motorSpeed, int kp, int kd, int threshold, int velocity, int delta) {
  long totalDriveTime = 1500;
   startArm = millis();
  motor.speed(ARM_MOTOR_OUTPUT, -motorSpeed);
  delay(200);
  long startDriving = millis();
  tapeFollowing(kp, kd, threshold, velocity, delta);
  while(millis() - startDriving < totalDriveTime) {
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if (digitalReadHighFilter(END_SWITCH_PIN))
      break;
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if ( (millis() - startArm) >= ARM_TIMEOUT )
      break; 
    tapeFollowing(kp, kd, threshold, velocity, delta);
  }
  if(millis() - startDriving > totalDriveTime) {
    velocity = 0;
    kp = 0;
    kd = 0;
    delta = 0;
  }
  while(TRUE) {
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if (digitalReadHighFilter(END_SWITCH_PIN))
      break;
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if ( (millis() - startArm) >= ARM_TIMEOUT )
      break; 
    tapeFollowing(kp, kd, threshold, velocity, delta);
  }
  motor.stop(ARM_MOTOR_OUTPUT);

  tapeFollowing(kp, kd, threshold, velocity, delta);
  startArm = millis();
  tapeFollowing(kp, kd, threshold, velocity, delta);
  motor.speed(ARM_MOTOR_OUTPUT, motorSpeed);
  tapeFollowing(kp, kd, threshold, velocity, delta);
   while(millis() - startDriving < totalDriveTime) {
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if (digitalReadHighFilter(START_SWITCH_PIN))
      break;
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if ( (millis() - startArm) >= ARM_TIMEOUT )
      break; 
    tapeFollowing(kp, kd, threshold, velocity, delta);
  }
  if(millis() - startDriving > totalDriveTime) {
    velocity = 0;
    kp = 0;
    kd = 0;
    delta = 0;
  }
  while(TRUE) {
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if (digitalReadHighFilter(START_SWITCH_PIN))
      break;
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if ( (millis() - startArm) >= ARM_TIMEOUT )
      break; 
    tapeFollowing(kp, kd, threshold, velocity, delta);
  }
  motor.stop(ARM_MOTOR_OUTPUT);
  
}

void armUpNoFollow(int motorSpeed) {
  startArm = millis();
  motor.speed(ARM_MOTOR_OUTPUT, -motorSpeed);
  while(TRUE) {
    if (digitalReadHighFilter(END_SWITCH_PIN))
      break;
    if ( (millis() - startArm) >= ARM_TIMEOUT )
      break; 
  }
  motor.stop(ARM_MOTOR_OUTPUT);
}

int armUpTurn(int motorSpeed, int kp, int kd, int threshold, int velocity, int delta, int turnSpeed) {
  int left;
  int right;
  long currentTime;
  long startArm;
  int checkForTape = FALSE;
  int checkTime = TRUE;
  int foundTape = FALSE;
  motor.speed(ARM_MOTOR_OUTPUT, -motorSpeed);  
  startArm = millis();
  delay(200);
  motor.speed(RIGHT_MOTOR_OUTPUT, -turnSpeed);
  motor.speed(LEFT_MOTOR_OUTPUT, turnSpeed);
  
  while(TRUE) {
    
    currentTime = millis();
    if (digitalReadHighFilter(END_SWITCH_PIN))
      break;
    if ( (currentTime - startArm) >= ARM_TIMEOUT )
      break; 
    if ( (currentTime - startArm >= 500) && (checkTime == TRUE)) {
      checkForTape = TRUE;
      checkTime  = FALSE;
    }
    if (checkForTape == TRUE) {
         left = analogRead(LEFT_QRD_INPUT);
         right = analogRead(RIGHT_QRD_INPUT);
         if ( (left < threshold) || (right < threshold)) {
           foundTape = TRUE;
           break;
         }
    }
  }
   while(foundTape == TRUE) {
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if (digitalReadHighFilter(END_SWITCH_PIN))
      break;
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if ( (millis() - startArm) >= ARM_TIMEOUT )
      break; 
    tapeFollowing(kp, kd, threshold, velocity, delta);
  }
  motor.stop(ARM_MOTOR_OUTPUT);
  return foundTape;
}

void armDownNoFollow(int motorSpeed) {
  startArm = millis();
  motor.speed(ARM_MOTOR_OUTPUT, motorSpeed);
   while(TRUE) {
     if (digitalReadHighFilter(START_SWITCH_PIN))
       break;
     if ( (millis() - startArm) >= ARM_TIMEOUT )
      break;
  }
  motor.stop(ARM_MOTOR_OUTPUT);
}

void armUp(int motorSpeed, int kp, int kd, int threshold, int velocity, int delta) {

  startArm = millis();
  motor.speed(ARM_MOTOR_OUTPUT, -motorSpeed);
  delay(200);
  tapeFollowing(kp, kd, threshold, velocity, delta);
  while(TRUE) {
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if (digitalReadHighFilter(END_SWITCH_PIN))
      break;
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if ( (millis() - startArm) >= ARM_TIMEOUT )
      break; 
    tapeFollowing(kp, kd, threshold, velocity, delta);
  }
  motor.stop(ARM_MOTOR_OUTPUT);
}

void armDown(int motorSpeed, int kp, int kd, int threshold, int velocity, int delta) {
  tapeFollowing(kp, kd, threshold, velocity, delta);
  startArm = millis();
  tapeFollowing(kp, kd, threshold, velocity, delta);
  motor.speed(ARM_MOTOR_OUTPUT, motorSpeed);
  tapeFollowing(kp, kd, threshold, velocity, delta);
   while(TRUE){
    tapeFollowing(kp, kd, threshold, velocity, delta);
     if (digitalReadHighFilter(START_SWITCH_PIN))
       break;
    tapeFollowing(kp, kd, threshold, velocity, delta);
     if ( (millis() - startArm) >= ARM_TIMEOUT )
      break; 
    tapeFollowing(kp, kd, threshold, velocity, delta);
  }
  motor.stop(ARM_MOTOR_OUTPUT);
}

void armDownABit() {
  motor.speed(ARM_MOTOR_OUTPUT, 600);
  delay(100);
  motor.stop(ARM_MOTOR_OUTPUT);
}


int armUpIR(int motorSpeed, int kp, int kd, int threshold, int velocity, int delta, int IR_velocity, int IR_kp, int IR_kd, int offIR, int IRcorrection, int beginIR) {
  int broken = FALSE;
  startArm = millis();
  motor.speed(ARM_MOTOR_OUTPUT, -motorSpeed);
  delay(200);
  tapeFollowing(kp, kd, threshold, velocity, delta);

  while(getAvgRightSignal() < beginIR) {
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if (digitalReadHighFilter(END_SWITCH_PIN)) {
      broken = TRUE;
      break;
  }
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if ( (millis() - startArm) >= ARM_TIMEOUT ) {
      broken = TRUE;
      break; 
    }
    tapeFollowing(kp, kd, threshold, velocity, delta);

  }

  while(broken == FALSE) {
    IRFollowing(IR_velocity, IR_kp, IR_kd, offIR, IRcorrection);
    if (digitalReadHighFilter(END_SWITCH_PIN)) {
      break;
  }
    IRFollowing(IR_velocity, IR_kp, IR_kd, offIR, IRcorrection);
    if ( (millis() - startArm) >= ARM_TIMEOUT ) {
      break; 
    }
    IRFollowing(IR_velocity, IR_kp, IR_kd, offIR, IRcorrection);

  }
  motor.stop(ARM_MOTOR_OUTPUT);
  return broken == FALSE;
}

int armDownIR(int motorSpeed, int kp, int kd, int threshold, int velocity, int delta, int IR_velocity, int IR_kp, int IR_kd, int offIR, int IRcorrection, int beginIR) {
  int broken = FALSE;
  startArm = millis();
  motor.speed(ARM_MOTOR_OUTPUT, motorSpeed);
  tapeFollowing(kp, kd, threshold, velocity, delta);
  while(getAvgRightSignal() < beginIR) {
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if (digitalReadHighFilter(START_SWITCH_PIN)) {
      broken = TRUE;
      break;
  }
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if ( (millis() - startArm) >= ARM_TIMEOUT ) {
      broken = TRUE;
      break; 
    }
    tapeFollowing(kp, kd, threshold, velocity, delta);

  }
  while(broken == FALSE) {
    IRFollowing(IR_velocity, IR_kp, IR_kd, offIR, IRcorrection);
    if (digitalReadHighFilter(START_SWITCH_PIN)) {
      break;
  }
    IRFollowing(IR_velocity, IR_kp, IR_kd, offIR, IRcorrection);
    if ( (millis() - startArm) >= ARM_TIMEOUT ) {
      break; 
    }
    IRFollowing(IR_velocity, IR_kp, IR_kd, offIR, IRcorrection);

  }
  motor.stop(ARM_MOTOR_OUTPUT);

  return broken == FALSE;
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
