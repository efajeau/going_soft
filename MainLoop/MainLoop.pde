/**THE ROBOT IS MEANT TO RUN THE WHOLE COURSE WITH THIS CODE
 * TUNING ONCE INITIALLY FOR ALL PARAMETERS
 * TUNING CAN BE IGNORED AND VALUES HARDCODED INSTEAD
 */

#include <phys253.h>      
#include <LiquidCrystal.h>    
#include <Servo253.h>

#define LEFT_QRD_INPUT 1
#define RIGHT_QRD_INPUT 2
#define LEFT_IR_INPUT 3
#define RIGHT_IR_INPUT 4
#define IR_THRESHOLD 150
#define START_SWITCH_PIN 0
#define END_SWITCH_PIN 1
#define ARTIFACT_DETECT_SWITCH 2
#define ARM_MOTOR_OUTPUT 2
#define TRUE 1
#define FALSE 0
#define TAPE_FOLLOWING 0
#define ARTIFACT_ARM 1
#define IR_SENSOR 2
#define REVERSE_DRIVING 3

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

int tapeValues[5] = {0, 0, 0, 0, 0};
int IRValues[3] = {0, 0, 0};
int testOptions[4] = {0, 0, 0, 0};

void tapeTuning(int vals[]);
void IRTuning(int vals[]);
int tuneArm();
void selectionMenu(int testOptions[]);
//void testTapeFollowing();
//void testIR();
//void testArm();

void setup() {
  portMode(0, INPUT);
  portMode(1, INPUT);
  
  pinMode(START_SWITCH_PIN, INPUT);
  pinMode(END_SWITCH_PIN, INPUT);
  pinMode(ARTIFACT_DETECT_SWITCH, INPUT);
  digitalWrite(ARTIFACT_DETECT_SWITCH, HIGH);
}

void loop() {
  
  selectionMenu(testOptions);
  
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
  
  while(!stopbutton()) {
    if (testOptions[TAPE_FOLLOWING] == TRUE) {
      tapeFollowing(kp, kd, threshold, velocity, delta);
    }
    if (testOptions[ARTIFACT_ARM] == TRUE && digitalRead(ARTIFACT_DETECT_SWITCH) == HIGH) {
      motor.stop(0);
      motor.stop(1);
      swingArm(armSpeed);
    }
    if (testOptions[IR_SENSOR] == TRUE && getIRSignal() > IR_THRESHOLD) {
      while(!stopbutton()) {
       IRFollowing(IR_velocity, IR_kp, IR_kd);
      }
    }
  }
}

/**
 * Initializes tuning for all parameters
 */
void tapeTuning(int vals[]) {
  //MENU TO SET PID
   while ( !(stopbutton()) ){
    vals[0] = knob(6);
    vals[1] = knob(7);
    LCD.home() ;
    LCD.setCursor(0,0); LCD.print("Tuning PID for tape");
    LCD.setCursor(0,0); LCD.print("P: "); LCD.print(vals[0]);
    LCD.setCursor(0,1);LCD.print("D: "); LCD.print(vals[1]);
    delay(10);
    LCD.clear(); 
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
  
  while (startbutton() ){}
  //MENU TO SET SPEED
  while( !(startbutton()) ){
    vals[3] = knob(6);
    vals[4] = floor(knob(7)/1023.0*5.0);
    if( vals[3] > 700 ){
      vals[3] = 700;
    }
    
    LCD.setCursor(0,0); LCD.print("Tuning speed and turning for tape");
    LCD.setCursor(0,1); LCD.print("SPEED: "); LCD.print(vals[3]);
    LCD.setCursor(8,1); LCD.print("DELTA: "); LCD.print(vals[4]);
    delay(10);
    LCD.clear();
  }
}

void IRTuning(int vals[]) {
  while ( !(stopbutton()) ){
    vals[0] = knob(6);
    vals[1] = knob(7);
    LCD.home() ;
    LCD.setCursor(0,0); LCD.print("Tuning PID for IR detection");
    LCD.setCursor(0,1); LCD.print("P: "); LCD.print(vals[0]);
    LCD.setCursor(8,1); LCD.print("D: "); LCD.print(vals[1]);
    delay(10);
    LCD.clear(); 
  }
  //MENU TO SET SPEED
  while( !(startbutton()) ){
    vals[2] = knob(6);
    if( vals[2] > 700 ){
      vals[2] = 700;
    }
    
    LCD.setCursor(0,0); LCD.print("Tuning speed for IR");
    LCD.setCursor(0,1); LCD.print("SPEED: "); LCD.print(vals[2]);
    delay(10);
    LCD.clear();
  }
}

int tuneArm() {
  
  int armSpeed = 0;
  
  while( !(stopbutton()) ) {
     armSpeed = knob(6);
     LCD.home();
     LCD.setCursor(0,0); LCD.print("ARM SPEED: "); LCD.print(armSpeed);
     delay(10);
     LCD.clear();
  }
  
  return armSpeed;
}

void selectionMenu(int testOptions[]) {
  while (!stopbutton()) {
    LCD.home();
    LCD.setCursor(0,0); LCD.print("WHAT TEST?");
  
    testSelect = floor(knob(6)/1023.0*4.0);
    LCD.home(); LCD.setCursor(0,1);
  
    if (testSelect == 0) {
      LCD.print("Tape Following");
      if (startbutton()) {
        if (testOptions[0] == FALSE) {
          testOptions[0] = TRUE;
          LCD.setCursor(14, 0); LCD.print("*");
        }
        else {
          testOptions[0] = FALSE;
          LCD.setCursor(14, 0); LCD.print(" ");
        }
      }
      delay(100);
    }
    else if (testSelect == 1) {
      LCD.print("Artifact Arm");
      if (startbutton()) {
        if (testOptions[1] == FALSE) {
          testOptions[1] = TRUE;
          LCD.setCursor(14, 0); LCD.print("*");
        }
        else {
          testOptions[1] = FALSE;
          LCD.setCursor(14, 0); LCD.print(" ");
        }
      }
      delay(100);
    }
    else if (testSelect == 2){
      LCD.print("IR Sensor");
      if (startbutton()) {
        if (testOptions[2] == FALSE) {
          testOptions[2] = TRUE;
          LCD.setCursor(14, 0); LCD.print("*");
        }
        else {
          testOptions[2] = FALSE;
          LCD.setCursor(14, 0); LCD.print(" ");
        }
      }
      delay(100);
    }
    else {
      LCD.print("Reverse Driving");
      if (startbutton()) {
        if (testOptions[3] == FALSE) {
          testOptions[3] = TRUE;
          LCD.setCursor(14, 0); LCD.print("*");
        }
        else {
          testOptions[3] = FALSE;
          LCD.setCursor(14, 0); LCD.print(" ");
        }
      }
      delay(100);
    }
    LCD.clear();
    delay(10);
  }
}
