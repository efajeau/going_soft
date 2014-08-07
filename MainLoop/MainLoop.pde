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


