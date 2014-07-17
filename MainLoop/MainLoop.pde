/**THE ROBOT IS MEANT TO RUN THE WHOLE COURSE WITH THIS CODE
 * TUNING ONCE INITIALLY FOR ALL PARAMETERS
 * TUNING CAN BE IGNORED AND VALUES HARDCODED INSTEAD
 */

#include <phys253.h>          //   ***** from 253 template file
#include <LiquidCrystal.h>    //   ***** from 253 template file
#include <Servo253.h>

#define LEFT_QRD_INPUT 1
#define RIGHT_QRD_INPUT 2
#define IR_THRESHOLD 150

int kp;
int kd;
int threshold;
int velocity;
int delta;

int[5] tuneValues = {0, 0, 0, 0, 0};

void setup() {
  
}

void loop() {
  
  /**FOR TUNING VALUES
   * CAN BE HARDCODED INSTEAD  
   */
  tuning(tuneValues);
  kp = tuneValues(0);
  kd = tuneValues(1);
  threshold = tuneValues(2)
  velocity = tuneValues(3);
  delta = tuneValues(4);
  
  //START WITH TAPE-FOLLOWING
  while(!stopbutton()) {
    tapeFollowing(kp, kd, threshold, velocity, delta);
    //SWING ARM WHILE ROLLING
    if (ARTIFACT_DETECTED_SWITCH = HIGH) {
      swingArm();
    }
    //DETECT IF ON ROCKS W/ THRESHOLD IR SIGNAL
    if (getIRSignal() > IR_THRESHOLD) {
      while(!stopbutton()) {
       IRFollowing(velocity, kp, kd);
      }
    }
  }
}

void tuning(int[] vals) {
  //MENU TO SET PID
   while ( !(stopbutton()) ){
   
    vals[0] = knob(6);
    vals[1] = knob(7);
    LCD.home() ;
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
    if( velocity > 700 ){
      velocity = 700;
    }
    
    LCD.setCursor(0,0); LCD.print("SPEED: "); LCD.print(vals[3]);
    LCD.setCursor(0,1); LCD.print("DELTA: "); LCD.print(vals[4]);
    delay(10);
    LCD.clear();
  }
}
