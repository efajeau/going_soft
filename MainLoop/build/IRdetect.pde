#include <phys253.h>         
#include <LiquidCrystal.h>   
#include <Servo253.h>     

#define LEFT_IR_INPUT 3
#define RIGHT_IR_INPUT 4
#define RIGHT_MOTOR_OUTPUT 0
#define LEFT_MOTOR_OUTPUT 1

#define TRUE 1;
#define FALSE 0;

double IR_error = 0;
int last_IR_error = 0;
int stored_IR_lerr;
int IR_ki;
int IR_p;
int IR_d;
//int IR_i;
int IR_pd;
int IR_time;
int IR_store_time;
int IR_threshold = 100;
long startTurning = 0;
int writeCount = 1;
int tolerance = 15;

int getIRSignal() {
  int leftIR = analogRead(LEFT_IR_INPUT);
  int rightIR = analogRead(RIGHT_IR_INPUT);

  return (leftIR + rightIR);
}

double getAverageIRSignal(double iterations) {
  double avgSignal = 0.0;
  int leftIR = 0;
  int rightIR = 0;

  for (int i = 0; i < iterations; i++) {
    leftIR = analogRead(LEFT_IR_INPUT);
    rightIR = analogRead(RIGHT_IR_INPUT);
    avgSignal += leftIR + rightIR;
  }

  avgSignal = avgSignal/iterations;

  return avgSignal;
}

int getAvgLeftSignal() {
  double avgSignal = 0.0;
  
  for (int i = 0; i < 5; i++) {
    avgSignal += analogRead(LEFT_IR_INPUT);
  }
  
  avgSignal = avgSignal/5.0;
  return floor(avgSignal);
}

int getAvgRightSignal() {
  double avgSignal = 0.0;
  
  for (int i = 0; i < 5; i++) {
    avgSignal += analogRead(RIGHT_IR_INPUT);
  }
  
  avgSignal = avgSignal/5.0;
  return floor(avgSignal);
}

int IRFollowing(int velocity, int kd, int kp, int offIR, int IRcorrection) {
  
  double denominator;
  double base = sqrt(250);
  //IR CORRECTION ALGORITHM

  int leftIR = getAvgLeftSignal() - 15;
  int rightIR = getAvgRightSignal();

   if ( (rightIR < offIR) && (leftIR < offIR) ) {
     if (last_IR_error < 0 ){
       IR_error = -IRcorrection;
       if (writeCount > 1000) {
          writeCount = 0;
          LCD.clear();
          LCD.home();
          LCD.print("neg cor");
       }
     }
     else if (last_IR_error >= 0){
       IR_error = IRcorrection;
        if (writeCount > 1000) {
          writeCount = 0;
          LCD.clear();
          LCD.home();
          LCD.print("pos cor");
       }
     }
   }
   else if ( (rightIR < offIR) && (leftIR  > offIR) ) { 
         IR_error = IRcorrection/2.0;
       if (writeCount > 1000) {
          writeCount = 0;
          LCD.clear();
          LCD.home();
          LCD.print("pos small");
       }
   }

   else if ( (leftIR < offIR) && (rightIR > offIR) ) { 
         IR_error = -IRcorrection/2.0;
       if (writeCount > 1000) {
          writeCount = 0;
          LCD.clear();
          LCD.home();
          LCD.print("neg small");
       }
   }
   else if ( abs(leftIR-rightIR) < tolerance) {
     IR_error = 0;
     if (writeCount > 1000) {
          writeCount = 0;
          LCD.clear();
          LCD.home();
          LCD.print("center");
     }
   }
   else if (leftIR - rightIR > 0) {
         IR_error = IRcorrection/4.0;
         denominator = leftIR+rightIR;
         denominator = sqrt(denominator);
         denominator = denominator/base;
         IR_error = IR_error/denominator;

   }
     else {
         IR_error = -IRcorrection/4.0;
         denominator = leftIR+rightIR;
         denominator = sqrt(denominator);
         denominator = denominator/base;
         IR_error = IR_error/denominator;
    
     }


   
   if (last_IR_error != IR_error ){
     stored_IR_lerr = last_IR_error;
     last_IR_error = IR_error;
      IR_store_time = IR_time;
      IR_time = 1;
   }
    
     IR_p = kp*IR_error;
     
     IR_d = (IR_error-stored_IR_lerr);
     IR_d = kd*IR_d;
     
     denominator = IR_store_time;
     denominator = denominator + IR_time;
     

     IR_d = IR_d/denominator;

     IR_pd = IR_p + IR_d;
     
    int thresh = 1020 - velocity;
    if (IR_pd > thresh) {
      IR_pd = thresh;
    } else if (IR_pd < -thresh) {
      IR_pd = -thresh;
    }
    int IR_pd_final = IR_pd;
    
    motor.speed(RIGHT_MOTOR_OUTPUT, velocity + IR_pd_final);
    motor.speed(LEFT_MOTOR_OUTPUT, velocity - IR_pd_final);
    IR_time = IR_time + 1;
    //writeCount++;
}

void rockTurning(int turnSpeed, int threshold) {
  motor.speed(RIGHT_MOTOR_OUTPUT, -turnSpeed);
  motor.speed(LEFT_MOTOR_OUTPUT, turnSpeed);
  
  startTurning = millis();
  while ( getAverageIRSignal(10.0) < threshold ) {
    if ( (millis() - startTurning) > 10000) {
      motor.speed(RIGHT_MOTOR_OUTPUT, 500);
      motor.speed(LEFT_MOTOR_OUTPUT, 500);
      delay(500);
      startTurning = millis();
    }
  } 
}
