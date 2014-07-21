/**
 * Tape following algorithm for an off-centre follow
 */

#include <phys253.h>          
#include <LiquidCrystal.h>
#include <Servo253.h>

#define LEFT_QRD_INPUT 3
#define RIGHT_QRD_INPUT 4
#define LEFT_MOTOR_OUTPUT 1
#define RIGHT_MOTOR_OUTPUT 0

int error;
int last_error;
int stored_lerr;
//int kp;
//int kd;
//int ki;                                   
int p;
int d;
//int i;
int pd;
int time;
int store_time;
//int threshold;
//int velocity;
int correction = 5;
//int delta = 0.0;

void tapeFollowing(int kp, int kd, int threshold, int velocity, int delta) {
  //TAPE FOLLOWING ALGORITHM

    int left = analogRead(LEFT_QRD_INPUT);
    int right = analogRead(RIGHT_QRD_INPUT);
    LCD.clear(); LCD.home();
    LCD.print("L: "); LCD.print(left); LCD.setCursor(8,0); LCD.print("R: "); LCD.print(right);
    
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
    
    motor.speed(RIGHT_MOTOR_OUTPUT, velocity+pd);
    motor.speed(LEFT_MOTOR_OUTPUT, velocity-pd);
    time = time + 1;
    
}
