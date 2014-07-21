#include <phys253.h>      
#include <LiquidCrystal.h>    
#include <Servo253.h>

//-----ANALOG INPUTS------//
#define LEFT_QRD_INPUT 1
#define RIGHT_QRD_INPUT 2
#define LEFT_IR_INPUT 3
#define RIGHT_IR_INPUT 4

//-----DIGITAL INPUTS-----//
#define START_SWITCH_PIN 0
#define END_SWITCH_PIN 3
#define ARTIFACT_DETECT_SWITCH 6
#define ARM_MOTOR_OUTPUT 2

#define TRUE 1
#define FALSE 0

#define TAPE_FOLLOWING 0
#define ARTIFACT_ARM 1
#define IR_SENSOR 2
#define REVERSE_DRIVING 3

//IR SIGNAL AT WHICH TO START CLIMBING
#define IR_THRESHOLD 150

#include "WProgram.h"
#include <HardwareSerial.h>
void setup();
void loop();
void tapeTuning(int vals[]);
void IRTuning(int vals[]);
int tuneArm();
void selectionMenu(int testOptions[]);
int getIRSignal();
void IRFollowing(int velocity, int kd, int kp);
void swingArm(int armSpeed);
void tapeFollowing(int kp, int kd, int threshold, int velocity, int delta);
int kp;
int kd;
int threshold;
int velocity;
int delta;
int IR_kp;
int IR_kd;
int IR_velocity;
int testSelect;
int armSpeed;
int tuning = TRUE;

int tapeValues[5] = {0, 0, 0, 0, 0};
int IRValues[3] = {0, 0, 0};
int testOptions[4] = {0, 0, 0, 0};

void tapeTuning(int vals[]);
void IRTuning(int vals[]);
int tuneArm();
void selectionMenu(int testOptions[]);

void setup() {
  Serial.begin(9600);
  portMode(0, INPUT);
  portMode(1, INPUT);
  
  pinMode(START_SWITCH_PIN, INPUT);
  pinMode(END_SWITCH_PIN, INPUT);
  pinMode(ARTIFACT_DETECT_SWITCH, INPUT);
  
  
}

void loop() {
  
  if (tuning == TRUE) {
  //Start by selection which items to test
  selectionMenu(testOptions);
  
  //Then tune for the selected items
    if (testOptions[TAPE_FOLLOWING] == TRUE) {
      tapeTuning(tapeValues);
      kp = tapeValues[0];
      kd = tapeValues[1];
      threshold = tapeValues[2];
      velocity = tapeValues[3];
      delta = tapeValues[4];
    }
  
    if (testOptions[ARTIFACT_ARM] == TRUE) {
      armSpeed = tuneArm();
    }
  
    if (testOptions[IR_SENSOR] == TRUE) {
      IRTuning(IRValues);
      IR_kp = IRValues[0];
      IR_kd = IRValues[1];
      IR_velocity = IRValues[2];
    }
  }
  
  //Operation of robot
  while(!stopbutton()) {
    if (testOptions[TAPE_FOLLOWING] == TRUE) {
      tapeFollowing(kp, kd, threshold, velocity, delta);
      Serial.println(velocity);
      Serial.println(testOptions[ARTIFACT_ARM]);
    }
    digitalWrite(ARTIFACT_DETECT_SWITCH, HIGH);
    if (testOptions[ARTIFACT_ARM] == TRUE && digitalRead(ARTIFACT_DETECT_SWITCH) == HIGH) {
      motor.stop(0);
      motor.stop(1);
      swingArm(armSpeed);
    }
    if (testOptions[IR_SENSOR] == TRUE && getIRSignal() > IR_THRESHOLD) {
      IRFollowing(IR_velocity, IR_kp, IR_kd);
    }
  }
  
  while(stopbutton()){
    motor.stop(0);
    motor.stop(1);
  }
  
  //Decide whether to keep parameters or retune
  while( TRUE != FALSE ) {
    LCD.home();
    LCD.setCursor(0,0); LCD.print("START: RETRY");
    LCD.setCursor(0,1); LCD.print("STOP: RETUNE");
  
    if (startbutton()) {
      tuning = FALSE;
      break;
    }
    if (stopbutton()) {
      tuning = TRUE;
      break;
    }
    
    delay(10);
    LCD.clear();
 }
 
 while(startbutton()){}
}

//Initializes tuning for tape parameters
void tapeTuning(int vals[]) {
  //MENU TO SET PID
   while ( !(startbutton()) ){
    vals[0] = knob(6);
    vals[1] = knob(7);
    LCD.home() ;
    LCD.setCursor(0,0); LCD.print("TAPE TUNING");
    LCD.setCursor(0,1); LCD.print("P: "); LCD.print(vals[0]);
    LCD.setCursor(8,1);LCD.print("D: "); LCD.print(vals[1]);
    delay(10);
    LCD.clear(); 
  }
  while(startbutton()){
    delay(50);
  }
  //MENU TO SET THRESHOLD FOR QRD
  while ( !(startbutton()) ){
    
    int left = analogRead(LEFT_QRD_INPUT);
    int right = analogRead(RIGHT_QRD_INPUT);
    vals[2] = knob(6);
    LCD.setCursor(0,1); LCD.print("Threshold: ");LCD.print(vals[2]);
    LCD.setCursor(0,0); LCD.print("L: ");LCD.print(left); LCD.print(" R: ");LCD.print(right);
    delay(100);
    LCD.clear();
  
  }
  while (startbutton() ){
    delay(50);
  }
  //MENU TO SET SPEED
  while( !(startbutton()) ){
    vals[3] = knob(6);
    vals[4] = floor(knob(7)/1023.0*5.0);
    if( vals[3] > 700 ){
      vals[3] = 700;
    }
    
    LCD.setCursor(0,0); LCD.print("SPEED: "); LCD.print(vals[3]);
    LCD.setCursor(0,1); LCD.print("DELTA: "); LCD.print(vals[4]);
    delay(10);
    LCD.clear();
  }
}

//Initializes tuning for IR parameters
void IRTuning(int vals[]) {
  while ( !(startbutton()) ){
    vals[0] = knob(6);
    vals[1] = knob(7);
    LCD.home() ;
    LCD.setCursor(0,0); LCD.print("Tuning PID for IR detection");
    LCD.setCursor(0,1); LCD.print("P: "); LCD.print(vals[0]);
    LCD.setCursor(8,1); LCD.print("D: "); LCD.print(vals[1]);
    delay(10);
    LCD.clear(); 
  }
  while(startbutton()){
    delay(50);
  }
  //MENU TO SET SPEED
  while( !(startbutton()) ){
    int leftIR = analogRead(LEFT_IR_INPUT);
    int rightIR = analogRead(RIGHT_IR_INPUT);
    
    vals[2] = knob(6);
    if( vals[2] > 700 ){
      vals[2] = 700;
    }
    
    
    LCD.setCursor(0,0); LCD.print("L: "); LCD.print(leftIR); LCD.print("R: "); LCD.print(rightIR);
    LCD.setCursor(0,1); LCD.print("SPEED: "); LCD.print(vals[2]);
    delay(10);
    LCD.clear();
  }
}

//Initializes tuning for arm parameters
int tuneArm() {
  
  int armSpeed = 0;
  
  while( !(startbutton()) ) {
     armSpeed = knob(6);
     LCD.home();
     LCD.setCursor(0,0); LCD.print("ARM SPEED: "); LCD.print(armSpeed);
     delay(10);
     LCD.clear();
  }
  while(startbutton()){
    delay(50);
  }
  
  return armSpeed;
}

/**
 * Menu to select which items to test
 * An item is selected with the stop button
 * A '*' indicates that the item is selected
 * Press the start button to continue to tuning
 */
void selectionMenu(int testOptions[]) {
  while (!startbutton()) {
    LCD.home();
    LCD.setCursor(0,0); LCD.print("WHAT TO TEST?");
  
    testSelect = floor(knob(6)/1023.0*4.0);
    LCD.home(); LCD.setCursor(0,1);
  
    if (testSelect == 0) {
      LCD.print("Tape Following");
      if (testOptions[0] == TRUE) {
        LCD.setCursor(15, 0); LCD.print("*");
        if (stopbutton()) {
          while(stopbutton()){}
          testOptions[0] = FALSE;
        }
      }
      else if (testOptions[0] == FALSE) {
        LCD.setCursor(15, 0); LCD.print(" ");
        if (stopbutton()) {
          while(stopbutton()){}
          testOptions[0] = TRUE;
        }
      }
      delay(300);
    }
    else if (testSelect == 1) {
      LCD.print("Artifact Arm");
      if (testOptions[1] == TRUE) {
        LCD.setCursor(15, 0); LCD.print("*");
        if (stopbutton()) {
          while(stopbutton()){}
          testOptions[1] = FALSE;
        }
      }
      else if (testOptions[1] == FALSE) {
        LCD.setCursor(15, 0); LCD.print(" ");
        if (stopbutton()) {
          while(stopbutton()){}
          testOptions[1] = TRUE;
        }
      }
      delay(100);
    }
    else if (testSelect == 2){
      LCD.print("IR Sensor");
      if (testOptions[2] == TRUE) {
        LCD.setCursor(15, 0); LCD.print("*");
        if (stopbutton()) {
          while(stopbutton()){}
          testOptions[2] = FALSE;
        }
      }
      else if (testOptions[2] == FALSE) {
        LCD.setCursor(15, 0); LCD.print(" ");
        if (stopbutton()) {
          while(stopbutton()){}
          testOptions[2] = TRUE;
        }
      }
      delay(100);
    }
    else {
      LCD.print("Reverse Driving");
      if (testOptions[3] == TRUE) {
        LCD.setCursor(15, 0); LCD.print("*");
        if (stopbutton()) {
           while(stopbutton()){}
           testOptions[3] = FALSE;
        }
      }
      else if (testOptions[3] == FALSE) {
        LCD.setCursor(15, 0); LCD.print(" ");
        if (stopbutton()) {
          while(stopbutton()){}
          testOptions[3] = TRUE;
        }
      }
      delay(10);
    }
    LCD.clear();
  }
  
  while(startbutton()){}
}
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
  
//  while(stopbutton())
//  { 
//    motor.stop(RIGHT_MOTOR_OUTPUT);
//    motor.stop(LEFT_MOTOR_OUTPUT);
//  }
}
#include <phys253.h>          
#include <LiquidCrystal.h>
#include <Servo253.h>

#define START_SWITCH_PIN 0
#define END_SWITCH_PIN 3
#define ARM_MOTOR_OUTPUT 2

void digitalCheck(int pinNum);

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
    motor.speed(ARM_MOTOR_OUTPUT, armSpeed);

  }
  
  LCD.clear();
  
  motor.stop(ARM_MOTOR_OUTPUT);

}

//void digitalCheck(int pinNum) {
//  digitalWrite(pinNum, HIGH);
//  if (digitalRead(START_SWITCH_PIN) == HIGH) {
//       delay(50);
//       digitalWrite(START_SWITCH_PIN, HIGH);
//       if (digitalRead(START_SWITCH_PIN) == HIGH) {
//       break;
//      }
//    }
//}
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
    
//  while(stopbutton())
//  { 
//    motor.stop(RIGHT_MOTOR_OUTPUT);
//    motor.stop(LEFT_MOTOR_OUTPUT);
//  }
}
