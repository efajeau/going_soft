#include <phys253.h>         
#include <LiquidCrystal.h>   
#include <Servo253.h>     

#define LEFT_IR_INPUT 1
#define RIGHT_IR_INPUT 2
#define RIGHT_MOTOR_OUTPUT 0
#define LEFT_MOTOR_OUTPUT 1
#define STOPPING_SIGNAL 1100

int IR_error;
int last_IR_error;
int stored_IR_lerr;
int IR_ki;
int IR_p;
int IR_d;
//int IR_i;
int IR_pd;
int IR_time;
int IR_store_time;

int getIRSignal() {
  int leftIR = analogRead(LEFT_IR_INPUT);
  int rightIR = analogRead(RIGHT_IR_INPUT);
  
  return (leftIR + rightIR);
}

void IRFollowing(int velocity, int kd, int kp) {
  //IR CORRECTION ALGORITHM
  while ( !(stopbutton()) ) {

    int leftIR = analogRead(LEFT_IR_INPUT);
    int rightIR = analogRead(RIGHT_IR_INPUT);
    LCD.clear(); LCD.home();
    LCD.print("L: "); LCD.print(leftIR); LCD.setCursor(8,0); LCD.print("R: "); LCD.print(rightIR);
    LCD.setCursor(0,1); LCD.print("ER: "); LCD.print(IR_error); 
    
    IR_error = (leftIR - rightIR);
    
    delay(10);

    if (last_IR_error != IR_error ){
      stored_IR_lerr = last_IR_error;
      last_IR_error = IR_error;
      IR_store_time = IR_time;
      IR_time = 1;
    }
     
    IR_p = kp*IR_error/sqrt(leftIR+rightIR);
    IR_d = kd*(IR_error - stored_IR_lerr)/(IR_store_time + IR_time);
    //IR_i = IR_ki*IR_error+IR_i;
    IR_pd = (IR_p + IR_d);
    
    LCD.setCursor(8,1); LCD.print("PD:"); LCD.print(IR_pd);
    
    motor.speed(RIGHT_MOTOR_OUTPUT, velocity+IR_pd);
    motor.speed(LEFT_MOTOR_OUTPUT, velocity-IR_pd);
    IR_time = IR_time + 1;
    
    if( (leftIR+rightIR) > STOPPING_SIGNAL) {
      LCD.clear();  LCD.home() ;
      LCD.setCursor(0,0); LCD.print("L: "); LCD.print(leftIR);
      LCD.setCursor(0,1); LCD.print("R: "); LCD.print(rightIR);
      LCD.setCursor(1,0); LCD.print("END OF PATH");
      
      motor.stop(RIGHT_MOTOR_OUTPUT);
      motor.stop(LEFT_MOTOR_OUTPUT);
    }
  }
  
  while(stopbutton())
  { 
    motor.stop(RIGHT_MOTOR_OUTPUT);
    motor.stop(LEFT_MOTOR_OUTPUT);
  }
}
