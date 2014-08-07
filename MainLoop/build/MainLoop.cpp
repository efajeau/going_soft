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

//-----MISC. CONSTANTS----//F
#define TRUE 1
#define FALSE 0
#define TAPE_FOLLOWING 0
#define ARTIFACT_ARM 1
#define IR_SENSOR 2
#define GO_HOME 3
#define RAMP_DELAY 4000
#define RAMP_SPEED 0
#define SLOW_DOWN 50

#define NUM_PARAMS 17

//----DEFAULT PARAMETERS---//
#include "WProgram.h"
#include <HardwareSerial.h>
void setup();
void loop();
void newTuningMenu(int vals[]);
void selectionMenu(int testOptions[]);
void setDefault();
void resetTempValues();
int checkStartButton();
int checkStopButton();
int getIRSignal();
double getAverageIRSignal(double iterations);
int getAvgLeftSignal();
int getAvgRightSignal();
int IRFollowing(int velocity, int kd, int kp, int offIR, int IRcorrection);
void rockTurning(int turnSpeed, int threshold);
void resetIRvals();
void swingArm(int armSpeed, int kp, int kd, int threshold, int velocity, int delta);
void swingArmTurning(int motorSpeed, int kp, int kd, int threshold, int velocity, int delta);
void armUpNoFollow(int motorSpeed);
int armUpTurn(int motorSpeed, int kp, int kd, int threshold, int velocity, int delta, int turnSpeed);
void armDownNoFollow(int motorSpeed);
void armUp(int motorSpeed, int kp, int kd, int threshold, int velocity, int delta);
void armDown(int motorSpeed, int kp, int kd, int threshold, int velocity, int delta);
void armDownABit();
int armUpIR(int motorSpeed, int kp, int kd, int threshold, int velocity, int delta, int IR_velocity, int IR_kp, int IR_kd, int offIR, int IRcorrection, int beginIR);
int armDownIR(int motorSpeed, int kp, int kd, int threshold, int velocity, int delta, int IR_velocity, int IR_kp, int IR_kd, int offIR, int IRcorrection, int beginIR);
int digitalReadHighFilter(int pin);
void setLastError();
void setSignPos();
void setLastTurnError(int lastError);
void tapeFollowing(int kp, int kd, int threshold, int velocity, int delta);
void turnAround(int turnSpeed, int threshold);
void sweep();
int kp = 120;
int kd = 60;
int threshold = 120;
int velocity = 225;
int delta = 0;
int IR_kp = 60;
int IR_kd = 60;
int IR_velocity = 400;
int armSpeed = 550;
int beginIR = 130;
int endIR = 2000;
int ramping = 0;
int turnSpeed = 400;
int turnCount = 4;
int goingHome = FALSE;
int returnIRStart = 200;
int returnIRStop = 1300;
int firstIteration = TRUE;
int offIR = 100;
int IRcorrection = 100;
int aboveIRthresh = FALSE;

int leftIRcheck = 0;
int rightIRcheck = 0;

//----TEMPORARY VALUES---//
long startSlowDown = 0;
long startSpeedUp = 0;
int testSelect;
int tuning = TRUE;
int count = 0;
int def = FALSE;
long startTimeBack = 0;

int followingTape = FALSE;
int artifactArm = FALSE;
int IRsensor = FALSE;
int goHome = FALSE;
int leftCheck = 0;
int rightCheck = 0;
long startWhite = 0;
long whiteTime = 0;
long IRtimeoutStart = 0;
int IRtimeout = FALSE;
long totalTime;

int tapeValues[5] = {0, 0, 0, 0, 0};
int IRValues[5] = {0, 0, 0, 0, 0};
int testOptions[4] = {0, 0, 0, 0};
int armParameters[1] = {0};
int returnParams[2] = {0, 0};
int returnIRParams[2] = {0, 0};

String paramNames[NUM_PARAMS] = {"kp", "kd", "thresh", "speed", "delta", "armSpeed", "IRkp", "IRkd", "IRSpeed", "beginIR", "endIR", "turnSpeed", "turnCount", "Leave", "offRocks", "offIR", "IRcorrection"};
int tuningVals[NUM_PARAMS] =    {165,   60,    290,      320,     1,       550,      60,     60,      300,       1000,      2000,     1000,        4,          100,     740,        80,    100};
double maxVals[NUM_PARAMS] = { 1023.0, 1023.0, 1023.0,  900.0,   5.0,     1023.0,    1023.0, 1023.0, 1023.0,    2000.0, 1023.0, 1023.0, 5.0, 1023.0, 2000.0, 1023.0, 1023.0};

void tapeTuning(int vals[]);
void IRTuning(int vals[]);
void tuneArm(int vals[]);
void selectionMenu(int testOptions[]);
void setDefault();
void resetTempValues();
void tuneGoHome();
void tuneHomeIR();
int checkStartButton();
int checkStopButton();
void newTuningMenu(int vals[]);

void setup() {
  portMode(0, INPUT);
  portMode(1, INPUT);
  
  pinMode(START_SWITCH_PIN, INPUT);
  pinMode(END_SWITCH_PIN, INPUT);
  pinMode(ARTIFACT_DETECT_SWITCH, INPUT);
}

void loop() {
  
  //Tuning Menu
  if (tuning == TRUE) {
    //Start by selecting which items to test
    selectionMenu(testOptions);
    followingTape =  testOptions[TAPE_FOLLOWING];
    artifactArm = testOptions[ARTIFACT_ARM];
    IRsensor = testOptions[IR_SENSOR];
    goHome = testOptions[GO_HOME];

    setDefault();
    newTuningMenu(tuningVals);
  }
  //Check To See If on Tape
  while(!checkStartButton()) {
    leftIRcheck = getAvgLeftSignal();
    rightIRcheck = getAvgRightSignal();
    LCD.home();
    LCD.setCursor(0,0); LCD.print("L: "); LCD.print(leftIRcheck); 
    LCD.setCursor(8,0); LCD.print("R: "); LCD.print(rightIRcheck);

    delay(50);
    LCD.clear();
  }while (startbutton()){delay(50);}

  while (!checkStartButton()) {
     leftCheck = analogRead(LEFT_QRD_INPUT);
     rightCheck = analogRead(RIGHT_QRD_INPUT);
     LCD.clear();
     LCD.home();
     LCD.print("L: ");
     LCD.print(leftCheck);
     LCD.setCursor(8,0);
     LCD.print("R: ");
     LCD.print(rightCheck);
     
   //  threshold = knob(6);
     LCD.setCursor(0,1);
     LCD.print("thresh: "); LCD.print(threshold);
     delay(50);
   }
  //Operation of robot
//    if (followingTape == TRUE)
//      armDown(armSpeed, kp, kd, threshold, velocity, delta);
//    else 
//      armDownNoFollow(armSpeed);
  armDownNoFollow(armSpeed);
  while(!checkStopButton()) {
    //Follow Tape
    
    if (followingTape == TRUE) {
      
      tapeFollowing(kp, kd, threshold, velocity+ramping, delta);
    }
    //Check For Artifacts
    digitalWrite(ARTIFACT_DETECT_SWITCH, HIGH);
    if (artifactArm == TRUE && digitalRead(ARTIFACT_DETECT_SWITCH) == HIGH) {

      //back up for 200 milliseconds
     if (count == 3) {
      motor.speed(RIGHT_MOTOR_OUTPUT, -(IR_velocity+100));
      motor.speed(LEFT_MOTOR_OUTPUT, -(IR_velocity+100));
     }
     else if (count == 2 || count == 0) {
      motor.speed(RIGHT_MOTOR_OUTPUT, -velocity+ramping);
      motor.speed(LEFT_MOTOR_OUTPUT, -velocity+ramping);
     }
     else {
      motor.speed(RIGHT_MOTOR_OUTPUT, -velocity+ramping);
      motor.speed(LEFT_MOTOR_OUTPUT, -velocity+ramping);
     }

      startTimeBack = millis();
      setLastError();

      
      if (count == 3) {
        totalTime = 100;
        ( (millis() - startTimeBack) < totalTime);
      }
      else {
        totalTime = 500;
        while( (millis() - startTimeBack) < totalTime) {
          setLastError();
        }
      }
      
      motor.stop(RIGHT_MOTOR_OUTPUT);
      motor.stop(LEFT_MOTOR_OUTPUT);
      if (count == turnCount-1) {
        armUpNoFollow(armSpeed);
        armDownABit();
      } 
      else if (count == 2 && turnCount == 4) {
       armUpIR(armSpeed, kp, kd, threshold, velocity+ramping, delta, IR_velocity, IR_kp, IR_kd, offIR, IRcorrection, beginIR);
       int haveIRlower = armDownIR(armSpeed, kp, kd, threshold, velocity+ramping, delta, IR_velocity, IR_kp, IR_kd, offIR, IRcorrection, beginIR);
       if (haveIRlower == TRUE) {
        LCD.clear();
        LCD.home();
        LCD.print("IR FOLLOWING");
        followingTape = FALSE;
        aboveIRthresh = TRUE;
        }
          //swingArm(armSpeed, kp, kd, threshold, velocity+ramping, delta);
      }
      else if (count == 0) {
     //   swingArm(armSpeed, kp, kd, threshold, velocity+ramping, delta);
       swingArmTurning(armSpeed, kp, kd, threshold, velocity+ramping, delta);
      }
      else {
        swingArm(armSpeed, kp, kd, threshold, velocity+ramping, delta);
      }
      count++;
    if (count == 1) {
      LCD.clear();
      LCD.home();
      LCD.print("turn1");
      long timingFirstTurn = millis();
      motor.speed(RIGHT_MOTOR_OUTPUT, 900);
      motor.speed(LEFT_MOTOR_OUTPUT, -300);
      while (millis() - timingFirstTurn < (long)1500);
      LCD.clear();
      setLastTurnError(5);
      tapeFollowing(kp, kd, threshold, velocity+ramping, delta);
      ramping = 100;
      
    }

//      if (count==1)
//        startSpeedUp = millis();
      if (count==2) {
        startSlowDown = millis();
      }
      
      if (count == 3 && turnCount == 4) {
        IRtimeoutStart = millis();
      }
    }
//    //Ramp up after 1 Artifact
//    if (count == 1 && ramping < RAMP_SPEED) {
//      if ( ((millis()-startSpeedUp) >= RAMP_DELAY)) {
//        ramping = RAMP_SPEED;
//      }
//    }

    //Slow Down after 2 Artifacts
    if (count == 2 && ramping > 0) {
      if ( ((millis()-startSlowDown)) >= 2000 ) {
        ramping = 0;
      }
    }
    
    if (count == 3 && turnCount == 4 && IRtimeout == FALSE) {
      if ( (millis() - IRtimeoutStart) > 1250 ) {
//        LCD.clear();
//        LCD.home();
//        LCD.print("IR FOLLOWING");
        followingTape = FALSE;
        IRtimeout = TRUE;
        IRtimeoutStart = 0;
      }
    }
    //IR Follow after 3 Artifacts & above threshold
    if (IRsensor == TRUE && (getAvgLeftSignal() >= beginIR || getAvgRightSignal() >= beginIR) && count >= 3 && aboveIRthresh == FALSE) {
      followingTape = FALSE;
//      LCD.clear();
//      LCD.home();
//      LCD.print("IR FOLLOWING");
      aboveIRthresh = TRUE;
    }
    if (IRsensor == TRUE && (aboveIRthresh == TRUE || IRtimeout == TRUE) ) {
      IRFollowing(IR_velocity, IR_kp, IR_kd, offIR, IRcorrection);
    }
    //Turn Around after set # of artifacts
    if (goHome == TRUE && count == turnCount) {
      artifactArm = FALSE;
      //velocity -= SLOW_DOWN;
      if (turnCount < 4) {
        turnAround(turnSpeed, threshold);
      }
      else {
        rockTurning(turnSpeed, returnIRStart);
        goingHome = TRUE;
      }
      beginIR = returnIRStart;
      goHome = FALSE;
    }
//    //Sweep for tape after getting off the rocks
//    if (goingHome == TRUE) {
//
//      if (analogRead(LEFT_QRD_INPUT) < threshold && analogRead(RIGHT_QRD_INPUT) < threshold) {
//        if (startWhite == 0) {
//          startWhite = millis();
//        }
//        else {
//          whiteTime = millis() - startWhite;
//        }
//      }
//      else {
//        whiteTime = 0;
//        startWhite = 0;
//      }
//    }
    if (goingHome == TRUE && (getAvgLeftSignal() > returnIRStop || getAvgRightSignal() > returnIRStop) ) {
      IRsensor = FALSE;
      followingTape = TRUE;
      setLastTurnError(0);
      sweep();
//      LCD.clear();
//      LCD.home();
//      LCD.print("TAPE FOLLOWING");
      tapeFollowing(kp, kd, threshold, velocity+ramping, delta);
//      LCD.clear();
//      LCD.home();
//      LCD.print("TAPE FOLLOWING");
      goingHome = FALSE;
    }
  }
  
  //Stop motors if stop button is pressed
  while(stopbutton()){
    motor.stop(RIGHT_MOTOR_OUTPUT);
    motor.stop(LEFT_MOTOR_OUTPUT);
    delay(50);
  } 
  
  //Decide whether to keep parameters or retune
  while( TRUE ) {
    LCD.home();
    LCD.setCursor(0,0); LCD.print("START: GO SOFT");
    LCD.setCursor(0,1); LCD.print("STOP: RETUNE");
    delay(10);
    LCD.clear();
  
    if (startbutton()) {
      tuning = FALSE;
      break;
    }
    if (stopbutton()) {
      tuning = TRUE;
      break;
    }
  }
  while(startbutton() || stopbutton()){delay(50);}
  resetTempValues();
  resetIRvals();
  setLastTurnError(0);
  setSignPos();
}

void newTuningMenu(int vals[]) {
  while (!startbutton()) {
    int i = floor(knob(6)/1023.0*NUM_PARAMS);
    LCD.home();
    LCD.setCursor(0,0); LCD.print(paramNames[i]);
    LCD.setCursor(0,1); LCD.print(vals[i]);
    delay(50);
    LCD.clear();
    if (stopbutton()) {
      while(stopbutton()){delay(50);}

      while(!startbutton()) {
        vals[i] = floor(knob(7)/1023.0*maxVals[i]);
        LCD.setCursor(0,0); LCD.print(paramNames[i]);
        LCD.setCursor(0,1); LCD.print(vals[i]);
        delay(50);
        LCD.clear();
      }
      while(startbutton()){delay(50);}
    }
  } while(startbutton()){delay(50);}

  kp = vals[0];
  kd = vals[1];
  threshold = vals[2];
  velocity = vals[3];
  delta = vals[4];
  armSpeed = vals[5];
  IR_kp = vals[6];
  IR_kd = vals[7];
  IR_velocity = vals[8];
  beginIR = vals[9];
  endIR = vals[10];
  turnSpeed = vals[11];
  turnCount = vals[12];
  returnIRStart = vals[13];
  returnIRStop = vals[14];
  offIR = vals[15];
  IRcorrection = vals[16];
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
      LCD.print("GO HOME");
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

//Sets the parameters to default when the Default option is selected
void setDefault() {
  kp = 120;
  kd = 60;
  threshold = 225;
  velocity = 225;
  delta = 0;
  IR_kp = 60;
  IR_kd = 60;
  IR_velocity = 450;
  armSpeed = 550;
  beginIR = 250;
  endIR = 2000;
  ramping = 0;
  count = 0;
  turnSpeed = 900;
  turnCount = 4;
  returnIRStart = 200;
  returnIRStop = 1300;
}

//Sets all temporary values back to initial
void resetTempValues() {
  LCD.print("RESETTING");
  delay(1000);
  LCD.clear();
  count = 0;
  ramping = 0;
  goingHome = FALSE;
  startSpeedUp = 0;
  startSlowDown = 0;
  startTimeBack = 0;
  aboveIRthresh = FALSE;
  startWhite = 0;
  whiteTime = 0;
  IRtimeout = FALSE;
  IRtimeoutStart = 0;

  followingTape =  testOptions[TAPE_FOLLOWING];
  artifactArm = testOptions[ARTIFACT_ARM];
  IRsensor = testOptions[IR_SENSOR];
  goHome = testOptions[GO_HOME];
}
int checkStartButton() {
  if (startbutton()) {
    delay(50);
    if (startbutton()) {
      return TRUE;
    }
  }
  return FALSE;
}

int checkStopButton() {
  if (stopbutton()) {
    delay(50);
    if (stopbutton()) {
      return TRUE;
    }
  }
  return FALSE;
}


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
     if (last_IR_error <= 0 ){
       IR_error = -IRcorrection;
       if (writeCount > 1000) {
          writeCount = 0;
          LCD.clear();
          LCD.home();
          LCD.print("neg cor");
       }
     }
     else if (last_IR_error > 0){
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
  LCD.clear();
  LCD.home();
  LCD.print("rock turning");
  startTurning = millis();
  while ( getAvgLeftSignal() < threshold || getAvgRightSignal() < threshold ) {
    if ( (millis() - startTurning) > 10000) {
      motor.speed(RIGHT_MOTOR_OUTPUT, 500);
      motor.speed(LEFT_MOTOR_OUTPUT, 500);
      delay(500);
      startTurning = millis();
    }
  } 
  LCD.clear();
}

void resetIRvals() {
  last_IR_error = 0;
  long startTurning = 0;
}
#include <phys253.h>          
#include <LiquidCrystal.h>
#include <Servo253.h>


#define START_SWITCH_PIN 0
#define END_SWITCH_PIN 3
#define ARM_MOTOR_OUTPUT 2

#define TRUE 1
#define FALSE 0
#define ARM_TIMEOUT 20000
#define ARM_DOWN_SPEED 600

long startArm = 0;

int digitalReadHighFilter(int pin);
void armDown();
void armUp(int motorSpeed);
void swingArm(int armSpeed);

void swingArm(int armSpeed, int kp, int kd, int threshold, int velocity, int delta) {
  
  tapeFollowing(kp, kd, threshold, velocity, delta);
  
  armUp(armSpeed, kp, kd, threshold, velocity, delta);

  tapeFollowing(kp, kd, threshold, velocity, delta);
  
  armDown(armSpeed, kp, kd, threshold, velocity, delta);
  
  tapeFollowing(kp, kd, threshold, velocity, delta);
  
  

}

void swingArmTurning(int motorSpeed, int kp, int kd, int threshold, int velocity, int delta) {
  long totalDriveTime = 1500;
   startArm = millis();
  motor.speed(ARM_MOTOR_OUTPUT, -motorSpeed);
  delay(200);
  long startDriving = millis();
  tapeFollowing(kp, kd, threshold, velocity, delta);
  while(millis() - startDriving < totalDriveTime) {
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if (digitalReadHighFilter(END_SWITCH_PIN))
      break;
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if ( (millis() - startArm) >= ARM_TIMEOUT )
      break; 
    tapeFollowing(kp, kd, threshold, velocity, delta);
  }
  if(millis() - startDriving > totalDriveTime) {
    velocity = 0;
    kp = 0;
    kd = 0;
    delta = 0;
  }
  while(TRUE) {
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if (digitalReadHighFilter(END_SWITCH_PIN))
      break;
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if ( (millis() - startArm) >= ARM_TIMEOUT )
      break; 
    tapeFollowing(kp, kd, threshold, velocity, delta);
  }
  motor.stop(ARM_MOTOR_OUTPUT);

  tapeFollowing(kp, kd, threshold, velocity, delta);
  startArm = millis();
  tapeFollowing(kp, kd, threshold, velocity, delta);
  motor.speed(ARM_MOTOR_OUTPUT, motorSpeed);
  tapeFollowing(kp, kd, threshold, velocity, delta);
   while(millis() - startDriving < totalDriveTime) {
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if (digitalReadHighFilter(START_SWITCH_PIN))
      break;
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if ( (millis() - startArm) >= ARM_TIMEOUT )
      break; 
    tapeFollowing(kp, kd, threshold, velocity, delta);
  }
  if(millis() - startDriving > totalDriveTime) {
    velocity = 0;
    kp = 0;
    kd = 0;
    delta = 0;
  }
  while(TRUE) {
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if (digitalReadHighFilter(START_SWITCH_PIN))
      break;
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if ( (millis() - startArm) >= ARM_TIMEOUT )
      break; 
    tapeFollowing(kp, kd, threshold, velocity, delta);
  }
  motor.stop(ARM_MOTOR_OUTPUT);
  
}

void armUpNoFollow(int motorSpeed) {
  startArm = millis();
  motor.speed(ARM_MOTOR_OUTPUT, -motorSpeed);
  while(TRUE) {
    if (digitalReadHighFilter(END_SWITCH_PIN))
      break;
    if ( (millis() - startArm) >= ARM_TIMEOUT )
      break; 
  }
  motor.stop(ARM_MOTOR_OUTPUT);
}

int armUpTurn(int motorSpeed, int kp, int kd, int threshold, int velocity, int delta, int turnSpeed) {
  int left;
  int right;
  long currentTime;
  long startArm;
  int checkForTape = FALSE;
  int checkTime = TRUE;
  int foundTape = FALSE;
  motor.speed(ARM_MOTOR_OUTPUT, -motorSpeed);  
  startArm = millis();
  delay(200);
  motor.speed(RIGHT_MOTOR_OUTPUT, -turnSpeed);
  motor.speed(LEFT_MOTOR_OUTPUT, turnSpeed);
  
  while(TRUE) {
    
    currentTime = millis();
    if (digitalReadHighFilter(END_SWITCH_PIN))
      break;
    if ( (currentTime - startArm) >= ARM_TIMEOUT )
      break; 
    if ( (currentTime - startArm >= 500) && (checkTime == TRUE)) {
      checkForTape = TRUE;
      checkTime  = FALSE;
    }
    if (checkForTape == TRUE) {
         left = analogRead(LEFT_QRD_INPUT);
         right = analogRead(RIGHT_QRD_INPUT);
         if ( (left < threshold) || (right < threshold)) {
           foundTape = TRUE;
           break;
         }
    }
  }
   while(foundTape == TRUE) {
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if (digitalReadHighFilter(END_SWITCH_PIN))
      break;
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if ( (millis() - startArm) >= ARM_TIMEOUT )
      break; 
    tapeFollowing(kp, kd, threshold, velocity, delta);
  }
  motor.stop(ARM_MOTOR_OUTPUT);
  return foundTape;
}

void armDownNoFollow(int motorSpeed) {
  startArm = millis();
  motor.speed(ARM_MOTOR_OUTPUT, motorSpeed);
   while(TRUE) {
     if (digitalReadHighFilter(START_SWITCH_PIN))
       break;
     if ( (millis() - startArm) >= ARM_TIMEOUT )
      break;
  }
  motor.stop(ARM_MOTOR_OUTPUT);
}

void armUp(int motorSpeed, int kp, int kd, int threshold, int velocity, int delta) {

  startArm = millis();
  motor.speed(ARM_MOTOR_OUTPUT, -motorSpeed);
  delay(200);
  tapeFollowing(kp, kd, threshold, velocity, delta);
  while(TRUE) {
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if (digitalReadHighFilter(END_SWITCH_PIN))
      break;
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if ( (millis() - startArm) >= ARM_TIMEOUT )
      break; 
    tapeFollowing(kp, kd, threshold, velocity, delta);
  }
  motor.stop(ARM_MOTOR_OUTPUT);
}

void armDown(int motorSpeed, int kp, int kd, int threshold, int velocity, int delta) {
  tapeFollowing(kp, kd, threshold, velocity, delta);
  startArm = millis();
  tapeFollowing(kp, kd, threshold, velocity, delta);
  motor.speed(ARM_MOTOR_OUTPUT, motorSpeed);
  tapeFollowing(kp, kd, threshold, velocity, delta);
   while(TRUE){
    tapeFollowing(kp, kd, threshold, velocity, delta);
     if (digitalReadHighFilter(START_SWITCH_PIN))
       break;
    tapeFollowing(kp, kd, threshold, velocity, delta);
     if ( (millis() - startArm) >= ARM_TIMEOUT )
      break; 
    tapeFollowing(kp, kd, threshold, velocity, delta);
  }
  motor.stop(ARM_MOTOR_OUTPUT);
}

void armDownABit() {
  motor.speed(ARM_MOTOR_OUTPUT, 600);
  delay(100);
  motor.stop(ARM_MOTOR_OUTPUT);
}


int armUpIR(int motorSpeed, int kp, int kd, int threshold, int velocity, int delta, int IR_velocity, int IR_kp, int IR_kd, int offIR, int IRcorrection, int beginIR) {
  int broken = FALSE;
  startArm = millis();
  motor.speed(ARM_MOTOR_OUTPUT, -motorSpeed);
  delay(200);
  tapeFollowing(kp, kd, threshold, velocity, delta);

  while(getAvgRightSignal() < beginIR) {
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if (digitalReadHighFilter(END_SWITCH_PIN)) {
      broken = TRUE;
      break;
  }
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if ( (millis() - startArm) >= ARM_TIMEOUT ) {
      broken = TRUE;
      break; 
    }
    tapeFollowing(kp, kd, threshold, velocity, delta);

  }

  while(broken == FALSE) {
    IRFollowing(IR_velocity, IR_kp, IR_kd, offIR, IRcorrection);
    if (digitalReadHighFilter(END_SWITCH_PIN)) {
      break;
  }
    IRFollowing(IR_velocity, IR_kp, IR_kd, offIR, IRcorrection);
    if ( (millis() - startArm) >= ARM_TIMEOUT ) {
      break; 
    }
    IRFollowing(IR_velocity, IR_kp, IR_kd, offIR, IRcorrection);

  }
  motor.stop(ARM_MOTOR_OUTPUT);
  return broken == FALSE;
}

int armDownIR(int motorSpeed, int kp, int kd, int threshold, int velocity, int delta, int IR_velocity, int IR_kp, int IR_kd, int offIR, int IRcorrection, int beginIR) {
  int broken = FALSE;
  startArm = millis();
  motor.speed(ARM_MOTOR_OUTPUT, motorSpeed);
  tapeFollowing(kp, kd, threshold, velocity, delta);
  while(getAvgRightSignal() < beginIR) {
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if (digitalReadHighFilter(START_SWITCH_PIN)) {
      broken = TRUE;
      break;
  }
     tapeFollowing(kp, kd, threshold, velocity, delta);
    if ( (millis() - startArm) >= ARM_TIMEOUT ) {
      broken = TRUE;
      break; 
    }
    tapeFollowing(kp, kd, threshold, velocity, delta);

  }
  while(broken == FALSE) {
    IRFollowing(IR_velocity, IR_kp, IR_kd, offIR, IRcorrection);
    if (digitalReadHighFilter(START_SWITCH_PIN)) {
      break;
  }
    IRFollowing(IR_velocity, IR_kp, IR_kd, offIR, IRcorrection);
    if ( (millis() - startArm) >= ARM_TIMEOUT ) {
      break; 
    }
    IRFollowing(IR_velocity, IR_kp, IR_kd, offIR, IRcorrection);

  }
  motor.stop(ARM_MOTOR_OUTPUT);

  return broken == FALSE;
}

int digitalReadHighFilter(int pin) {
  digitalWrite(pin, HIGH);
    if (digitalRead(pin) == HIGH) {
       delay(10);
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
#define POSITIVE 1
#define NEGATIVE -1

int error;
int last_error = 0;
int stored_lerr;                                 
int p;
int d;
int pd;
int time;
int store_time;
int correction = 5;
int sign = POSITIVE;
long startTime = 0;
long turnSweepTime = 500;


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
      if (last_error >= 0){
        error = (correction + delta);
      }
    }
    if (last_error != error ){
      stored_lerr = last_error;
      last_error = error;
      store_time = time;
      time = 1;
    }
    time = time + 1;

}

void setSignPos() {
  sign = POSITIVE;
  turnSweepTime = 2000;
}
void setLastTurnError(int lastError) {
  last_error = lastError;
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
    
    int pdCap = 1020 - velocity;
    if (pd < -pdCap) {
      pd = -pdCap;
    }
    else if (pd > pdCap) {
      pd = pdCap;
    }
    
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
//  LCD.clear();
//  LCD.home();
//  LCD.setCursor(0,0);
 // LCD.print("SWEEPING");
  while ( (left < threshold) && (right < threshold)) {
    left = analogRead(LEFT_QRD_INPUT);
    right = analogRead(RIGHT_QRD_INPUT);
    
    if ( (millis() - startTime) > turnSweepTime)  {
      if (sign == POSITIVE) {
        sign = NEGATIVE;
        turnSweepTime = 500; 
      }
      else if (sign == NEGATIVE) {
        sign = POSITIVE;
        turnSweepTime = 500;
      }

      startTime = millis();
      
      motor.speed(RIGHT_MOTOR_OUTPUT, sign*(600) );
      motor.speed(LEFT_MOTOR_OUTPUT, sign*(-600));
    }
  }
//  LCD.clear();
}


