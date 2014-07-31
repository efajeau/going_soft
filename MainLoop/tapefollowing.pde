/**
 * Tape following algorithm for an off-centre follow
 */

#include <phys253.h>          
#include <LiquidCrystal.h>
#include <Servo253.h>

#define LEFT_QRD_INPUT 1
#define RIGHT_QRD_INPUT 2

#define LEFT_MOTOR_OUTPUT 1
#define RIGHT_MOTOR_OUTPUT 0
#define POSITIVE 1
#define NEGATIVE -1

int error;
int last_error = 5;
int stored_lerr;                                 
int p;
int d;
int pd;
int time;
int store_time;
int correction = 5;
int sign = POSITIVE;
long startTime = 0;


int offTape = FALSE;


void setLastError() {

  int left = analogRead(LEFT_QRD_INPUT);
  int right = analogRead(RIGHT_QRD_INPUT);
  
    if ( (right > threshold) && (left > threshold) ){ error = 0;}
    if ( (right < threshold) && (left  > threshold) ) { error = 1; }
    if ( (left < threshold) && (right > threshold) ) { error = -1;}
    if ( (right < threshold) && (left < threshold) ) {
      if (last_error < 0 ){
        error = -(correction);
      }
      if (last_error > 0){
        error = (correction + delta);
      }
    }
    if (last_error != error ){
      stored_lerr = last_error;
      last_error = error;
      store_time = time;
      time = 1;
    }
     

}

void tapeFollowing(int kp, int kd, int threshold, int velocity, int delta) {
  //TAPE FOLLOWING ALGORITHM
    int left = analogRead(LEFT_QRD_INPUT);
    int right = analogRead(RIGHT_QRD_INPUT);
    
    if ( (right > threshold) && (left > threshold) ){ error = 0;}
    if ( (right < threshold) && (left  > threshold) ) { error = 1; }
    if ( (left < threshold) && (right > threshold) ) { error = -1;}
    if ( (right < threshold) && (left < threshold) ) {
      if (last_error < 0 ){
        error = -(correction);
      }
      if (last_error > 0){
        error = (correction + delta);
      }
    }
    if (last_error != error ){
      stored_lerr = last_error;
      last_error = error;
      store_time = time;
      time = 1;
    }
     
    p = kp*error;
    d = kd*(error - stored_lerr)/(store_time + time);
    //i = ki*error+i;
    
    pd = p + d;
    
    motor.speed(RIGHT_MOTOR_OUTPUT, (velocity+pd));
    motor.speed(LEFT_MOTOR_OUTPUT, (velocity-pd));
    time = time + 1; 
}

void turnAround(int turnSpeed, int threshold) {
  
  motor.speed(RIGHT_MOTOR_OUTPUT, -turnSpeed);
  motor.speed(LEFT_MOTOR_OUTPUT, turnSpeed);
  
  delay(500);
  
  int left = analogRead(LEFT_QRD_INPUT);
  int right = analogRead(RIGHT_QRD_INPUT);
  while ( (left < threshold) || (right < threshold)) {
    left = analogRead(LEFT_QRD_INPUT);
    right = analogRead(RIGHT_QRD_INPUT);
  } 
}

void sweep() {
  int left = analogRead(LEFT_QRD_INPUT);
  int right = analogRead(RIGHT_QRD_INPUT);

  startTime = millis();
  LCD.home();
  LCD.setCursor(0,0);
  LCD.print("SWEEPING");
  while ( (left < threshold) && (right < threshold)) {
    left = analogRead(LEFT_QRD_INPUT);
    right = analogRead(RIGHT_QRD_INPUT);

    motor.speed(RIGHT_MOTOR_OUTPUT, sign*(600) );
    motor.speed(LEFT_MOTOR_OUTPUT, sign*(-600));

    if ( ((millis() - startTime) % 2000) == 0 )  {
      if (sign == POSITIVE)
        sign = NEGATIVE;
      else if (sign == NEGATIVE)
        sign = POSITIVE;
    }
  }
  LCD.clear();
}

