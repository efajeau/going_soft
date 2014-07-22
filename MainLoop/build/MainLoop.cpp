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

//-----MOTOR OUTPUTS------//
#define RIGHT_MOTOR_OUTPUT 0
#define LEFT_MOTOR_OUTPUT 1
#define ARM_MOTOR_OUTPUT 2

//-----MISC. CONSTANTS----//
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
void tuneArm(int vals[]);
void selectionMenu(int testOptions[]);
int getIRSignal();
void IRFollowing(int velocity, int kd, int kp, int forwards);
void swingArm(int armSpeed);
int digitalReadHighFilter(int pin);
void tapeFollowing(int kp, int kd, int threshold, int velocity, int delta, int forwards);
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
int count = 0;
int countingArtifacts = FALSE;
int forwards = TRUE;
int maxAmplitude = 800;
int countBackUp = 0;
int backUp = 100;
int maxTries = 2;
int tries = 0;

int tapeValues[5] = {0, 0, 0, 0, 0};
int IRValues[4] = {0, 0, 0, 0};
int testOptions[4] = {0, 0, 0, 0};
int armParameters[2] = {0, 0};

void tapeTuning(int vals[]);
void IRTuning(int vals[]);
void tuneArm(int vals[]);
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
  //Start by selecting which items to test
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
      tuneArm(armParameters);
      armSpeed = armParameters[0];
      countingArtifacts = armParameters[1];
    }
  
    if (testOptions[IR_SENSOR] == TRUE) {
      IRTuning(IRValues);
      IR_kp = IRValues[0];
      IR_kd = IRValues[1];
      IR_velocity = IRValues[2];
      maxAmplitude = IRValues[3];
    }
  }
  
  //Operation of robot
  while(!stopbutton()) {
    
    if (testOptions[IR_SENSOR] == TRUE) {
      if ((analogRead(LEFT_IR_INPUT)+analogRead(RIGHT_IR_INPUT)) > maxAmplitude*2) {
        forwards = FALSE;
        countBackUp = 0;
      }
      else if (countBackUp > backUp) {
        forwards = TRUE;
        tries++;
      }
      else {
        countBackUp++;
      }
    }
    if (testOptions[ARTIFACT_ARM] == TRUE) {
      if (count == 3 && countingArtifacts || tries == maxTries) {
          forwards = FALSE;
      }
    }
    
    if (testOptions[TAPE_FOLLOWING] == TRUE) {
      tapeFollowing(kp, kd, threshold, velocity, delta, forwards);
    }
    digitalWrite(ARTIFACT_DETECT_SWITCH, HIGH);
    if (testOptions[ARTIFACT_ARM] == TRUE && digitalRead(ARTIFACT_DETECT_SWITCH) == HIGH) {
      motor.stop(RIGHT_MOTOR_OUTPUT);
      motor.stop(LEFT_MOTOR_OUTPUT);
      swingArm(armSpeed);
      count++;
    }
    if (testOptions[IR_SENSOR] == TRUE && getIRSignal() > IR_THRESHOLD) {
      testOptions[TAPE_FOLLOWING] = FALSE;
      IRFollowing(IR_velocity, IR_kp, IR_kd, forwards);
    }
  }
  
  while(stopbutton()){
    motor.stop(RIGHT_MOTOR_OUTPUT);
    motor.stop(LEFT_MOTOR_OUTPUT);
    delay(50);
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
 while(startbutton() || stopbutton()){delay(50);}
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
  while(startbutton()){delay(50);}
  
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
  while (startbutton()) {delay(50);}
  
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
  while (startbutton()){delay(50);}
  
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
  while(startbutton()){delay(50);}
  
  //MENU TO SET SPEED
  while( !(startbutton()) ){
    int leftIR = analogRead(LEFT_IR_INPUT);
    int rightIR = analogRead(RIGHT_IR_INPUT);
    
    vals[2] = knob(6);
    vals[3] = knob(7);
    if( vals[2] > 700 ){
      vals[2] = 700;
    }
    
    
    LCD.setCursor(0,0); LCD.print("L: "); LCD.print(leftIR); LCD.print("R: "); LCD.print(rightIR);
    LCD.setCursor(0,1); LCD.print("S: "); LCD.print(vals[2]);
    LCD.setCursor(8,1); LCD.print("max: "); LCD.print(vals[3]);
    delay(10);
    LCD.clear();
  }
  while(startbutton()){delay(50);}
  
}

//Initializes tuning for arm parameters
void tuneArm(int vals[]) {
  
  while( !(startbutton()) ) {
     vals[0] = knob(6);
     if (stopbutton()) {
       delay(50);
       if (stopbutton()) {
         if (vals[1]) {
           vals[1] = FALSE;
         }
         else {
           vals[1] = TRUE;
         }
       }
     }
     LCD.clear();
     LCD.home();
     LCD.setCursor(0,0); LCD.print("ARM SPEED: "); LCD.print(vals[0]);
     LCD.setCursor(0,1); LCD.print("Counting: ");
     if (vals[1]) {
       LCD.print("T");
     }
     else {
       LCD.print("F");
     }
     delay(10);
     LCD.clear();
  }
  while(startbutton()){delay(50);}
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
  while(startbutton()){delay(50);}
}
#include <phys253.h>         
#include <LiquidCrystal.h>   
#include <Servo253.h>     

#define LEFT_IR_INPUT 3
#define RIGHT_IR_INPUT 4
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

void IRFollowing(int velocity, int kd, int kp, int forwards) {
  int sign;
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
    
    if (forwards) {
      sign = 1;
    }
    else {
      sign = -1;
    }
    
    motor.speed(RIGHT_MOTOR_OUTPUT, sign*(velocity+IR_pd));
    motor.speed(LEFT_MOTOR_OUTPUT, sign*(velocity-IR_pd));
    IR_time = IR_time + 1;
    
//    if( (leftIR+rightIR) > STOPPING_SIGNAL) {
//      LCD.clear();  LCD.home() ;
//      LCD.setCursor(0,0); LCD.print("L: "); LCD.print(leftIR);
//      LCD.setCursor(0,1); LCD.print("R: "); LCD.print(rightIR);
//      LCD.setCursor(1,0); LCD.print("END OF PATH");
//      
//      motor.stop(RIGHT_MOTOR_OUTPUT);
//      motor.stop(LEFT_MOTOR_OUTPUT);
//    }
  
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

void tapeFollowing(int kp, int kd, int threshold, int velocity, int delta, int forwards) {
  //TAPE FOLLOWING ALGORITHM
    int sign;
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

