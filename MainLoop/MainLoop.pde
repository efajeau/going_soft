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

int kp;
int kd;
int threshold;
int velocity;
int delta;
int IR_kp;
int IR_kd;
int IR_velocity;

int[5] tapeValues = {0, 0, 0, 0, 0};
int[3] IRValues = {0, 0, 0};

void setup() {
  portMode(0, INPUT);
  portMode(1, INPUT);
  
  pinMode(START_SWITCH_PIN, INPUT);
  pinMode(END_SWITCH_PIN, INPUT);
  pinMode(ARTIFACT_DETECT_SWITCH, INPUT);
  digitalWrite(ARTIFACT_DETECT_SWITCH, HIGH);
}

void loop() {
  
  /**FOR TUNING VALUES
   * CAN BE HARDCODED INSTEAD  
   */
  tapeTuning(tapeValues);
  kp = tapeValues(0);
  kd = tapeValues(1);
  threshold = tuneValues(2)
  velocity = tuneValues(3);
  delta = tuneValues(4);
  
  IRtuning(IRValues);
  IR_kp = IRValues(0);
  IR_kd = IRValues(1);
  IR_velocity = IRValues(2);
  
  //START WITH TAPE-FOLLOWING
  while(!stopbutton()) {
    tapeFollowing(kp, kd, threshold, velocity, delta);
    //SWING ARM WHILE ROLLING
    if (digitalRead(ARTIFACT_DETECT_SWITCH) == HIGH) {
      swingArm();
    }
    //DETECT IF ON ROCKS W/ THRESHOLD IR SIGNAL
    if (getIRSignal() > IR_THRESHOLD) {
      while(!stopbutton()) {
       IRFollowing(IR_velocity, IR_kp, IR_kd);
      }
    }
  }
}

/**
 * Initializes tuning for all parameters
 */
void tapeTuning(int[] vals) {
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

void IRTuning(int[] vals) {
  while ( !(stopbutton()) ){
    vals[0] = knob(6);
    vals[1] = knob(7);
    LCD.home() ;
    LCD.setCursor(0,0); LCD.print("Tuning PID for IR detection");
    LCD.setCursor(0,1); LCD.print("P: "); LCD.print(vals[0]);
    LCD.setCursor(8,1);LCD.print("D: "); LCD.print(vals[1]);
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
