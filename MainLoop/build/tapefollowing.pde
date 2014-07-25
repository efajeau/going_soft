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

int error;
int last_error = 5;
int stored_lerr;                                 
int p;
int d;
int pd;
int time;
int store_time;
int correction = 5;


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


void tapeFollowing(int kp, int kd, int threshold, int velocity, int delta, int forwards) {
  //TAPE FOLLOWING ALGORITHM
    int sign;
    int left = analogRead(LEFT_QRD_INPUT);
    int right = analogRead(RIGHT_QRD_INPUT);
//    LCD.clear(); LCD.home();
//    LCD.print("L: "); LCD.print(left); LCD.setCursor(8,0); LCD.print("R: "); LCD.print(right);
    
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
    
    if (forwards) {
      sign = 1;
    }
    else {
      sign = -1;
    }
    
    motor.speed(RIGHT_MOTOR_OUTPUT, sign*(velocity+pd));
    motor.speed(LEFT_MOTOR_OUTPUT, sign*(velocity-pd));
    time = time + 1; 
}

void turnAround(int turnSpeed, int turnDiff, int threshold) {
  
  motor.speed(RIGHT_MOTOR_OUTPUT, -turnSpeed - turnDiff);
  motor.speed(LEFT_MOTOR_OUTPUT, turnSpeed + turnDiff);
  
  delay(500);
  
  int left = analogRead(LEFT_QRD_INPUT);
  int right = analogRead(RIGHT_QRD_INPUT);
  while ( (left < threshold) || (right < threshold)) {
    left = analogRead(LEFT_QRD_INPUT);
    right = analogRead(RIGHT_QRD_INPUT);
  } 
}
