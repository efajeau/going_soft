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
#define GO_HOME 3
#define RAMP_DELAY 3000
#define RAMP_SPEED 100
#define SLOW_DOWN 0

//----DEFAULT PARAMETERS---//
int kp = 120;
int kd = 60;
int threshold = 120;
int velocity = 200;
int delta = 0;
int IR_kp = 120;
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

long startSlowDown = 0;
long startSpeedUp = 0;
int testSelect;
int tuning = TRUE;
int count = 0;
int countBackUp = 0;
int backUp = 100;
int maxTries = 2;
int tries = 0;
int def = FALSE;
int initVel = 0;
int tempIRStart = 0;

int tapeValues[5] = {0, 0, 0, 0, 0};
int IRValues[5] = {0, 0, 0, 0, 0};
int testOptions[4] = {0, 0, 0, 0};
int armParameters[1] = {0};
int returnParams[2] = {0, 0};
int returnIRParams[2] = {0, 0};

void tapeTuning(int vals[]);
void IRTuning(int vals[]);
void tuneArm(int vals[]);
void selectionMenu(int testOptions[]);
void setDefault();
void tuneGoHome();
void tuneHomeIR();

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
    
    while (true) {
      LCD.home();
      LCD.setCursor(0,0); LCD.print("DEFAULT VALUES?");
      delay(20);
      LCD.clear();
      if (startbutton()) {
        def = TRUE;
        setDefault();
        break;
      }
      if (stopbutton()) {
        def = FALSE;
        break;
      }
    }
    while(startbutton() || stopbutton()){delay(50);}
    
    
    if (def == FALSE) {
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
      }
  
      if (testOptions[IR_SENSOR] == TRUE) {
        IRTuning(IRValues);
        IR_kp = IRValues[0];
        IR_kd = IRValues[1];
        IR_velocity = IRValues[2];
        beginIR = IRValues[3];
        endIR = IRValues[4];
      }
      
      if (testOptions[GO_HOME] == TRUE) {
        tuneGoHome(returnParams);
        turnSpeed = returnParams[0];
        turnCount = returnParams[1];
      }

      if (testOptions[GO_HOME] == TRUE && testOptions[IR_SENSOR] == TRUE) {
        tuneHomeIR(returnIRParams);
        returnIRStart = returnIRParams[0];
        returnIRStop = returnIRParams[1];
      }
   }
  }
   int leftCheck;
   int rightCheck;
   while (!checkStartButton()) {
     leftCheck = analogRead(LEFT_QRD_INPUT);
     rightCheck = analogRead(RIGHT_QRD_INPUT);
     LCD.clear();
     LCD.home();
     LCD.print("L: ");
     LCD.print(leftCheck);
     LCD.setCursor(0,1);
     LCD.print("R: ");
     LCD.print(rightCheck);
     delay(20);
   }
  //Operation of robot
  //armDown();
  while(!checkStopButton()) {
    if (testOptions[TAPE_FOLLOWING] == TRUE) {
      tapeFollowing(kp, kd, threshold, velocity+ramping, delta);
    }
    digitalWrite(ARTIFACT_DETECT_SWITCH, HIGH);
    if (testOptions[ARTIFACT_ARM] == TRUE && digitalRead(ARTIFACT_DETECT_SWITCH) == HIGH) {
      motor.stop(RIGHT_MOTOR_OUTPUT);
      motor.stop(LEFT_MOTOR_OUTPUT);
      swingArm(armSpeed);
      count++;
      if (count==1)
        startSpeedUp = millis();
      if (count==2) {
        startSlowDown = millis();
      }
    }
    if (count == 1 && ramping < RAMP_SPEED) {
      if ( ((millis()-startSpeedUp) >= RAMP_DELAY)) {
        ramping = RAMP_SPEED;
      }
    }
    else if (count == 2 && ramping > 0) {
      if ( ((millis()-startSlowDown)) >= RAMP_DELAY )
        ramping = 0;
    }
    if (testOptions[GO_HOME]==TRUE && count == turnCount) {
      testOptions[ARTIFACT_ARM] = FALSE;
      initVel = velocity;
      velocity -= SLOW_DOWN;
      //turnAround(turnSpeed, threshold);
      rockTurning(turnSpeed, returnIRStart);
      tempIRStart = beginIR;
      beginIR = returnIRStart;
      goingHome = TRUE;
//      if (count == 4) {
//        goingHome = TRUE;
//        testOptions[IR_SENSOR] = FALSE;
//        rockTurning(turnSpeed, beginIR);
//        testOptions[IR_SENSOR] = TRUE;
//      }
      testOptions[GO_HOME] = FALSE;
    }
    if (goingHome == TRUE && getIRSignal() > returnIRStop) {
      testOptions[IR_SENSOR] = FALSE;
      testOptions[TAPE_FOLLOWING] = TRUE;
      sweep();
      
    }

    if (testOptions[IR_SENSOR] == TRUE && getIRSignal() >= beginIR && count >= 3) {
      testOptions[TAPE_FOLLOWING] = FALSE;
      IRFollowing(IR_velocity, IR_kp, IR_kd);
      // if ( goingHome == FALSE && getIRSignal() >= endIR) {
      //   motor.stop(RIGHT_MOTOR_OUTPUT);
      //   motor.stop(LEFT_MOTOR_OUTPUT);
      //   break;
      // }
    }
  }
  
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
 velocity = initVel;
 count = 0;
 ramping = 0;
 goingHome = FALSE;
 startSpeedUp = 0;
 startSlowDown = 0;
 beginIR = tempIRStart;
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
    if( vals[2] > 700 ){
      vals[2] = 700;
    }
    
    
    LCD.setCursor(0,0); LCD.print("L: "); LCD.print(leftIR); LCD.print("R: "); LCD.print(rightIR);
    LCD.setCursor(0,1); LCD.print("SPEED: "); LCD.print(vals[2]);
    delay(10);
    LCD.clear();
  }
  while(startbutton()){delay(50);}
  
  while(!(startbutton()) ) {
    vals[3] = knob(6);
    vals[4] = floor(knob(7)/1023.0*2000.0);
    
    LCD.home();
    LCD.setCursor(0,0); LCD.print("BEGIN: "); LCD.print(vals[3]);
    LCD.setCursor(0,1); LCD.print("END: "); LCD.print(vals[4]);
    delay(30);
    LCD.clear();
  }
  while(startbutton()){delay(50);}
}

void tuneHomeIR(int vals[]) {
  while(!(startbutton()) ) {
    vals[0] = knob(6);
    vals[1] = floor(knob(7)/1023.0*2000.0);
    
    LCD.home();
    LCD.setCursor(0,0); LCD.print("LEAVING: "); LCD.print(vals[0]);
    LCD.setCursor(0,1); LCD.print("OFF ROCKS: "); LCD.print(vals[1]);
    delay(30);
    LCD.clear();
  }
  while(startbutton()){delay(50);}
}

//Initializes tuning for arm parameters
void tuneArm(int vals[]) {
  while( !(startbutton()) ) {
     vals[0] = knob(6);

     LCD.clear();
     LCD.home();
     LCD.setCursor(0,0); LCD.print("ARM SPEED: "); LCD.print(vals[0]);
     delay(10);
     LCD.clear();
  }
  while(startbutton()){delay(50);}
}

void tuneGoHome(int vals[]) {
  while(!startbutton()) {
     vals[0] = knob(6);
     vals[1] = floor(knob(7)/1023.0*5.0);
     
     LCD.home();
     LCD.setCursor(0,0); LCD.print("Turn: "); LCD.print(vals[0]);
     LCD.setCursor(0,1); LCD.print("Count: "); LCD.print(vals[1]);
     
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

void setDefault() {
  kp = 120;
  kd = 60;
  threshold = 200;
  velocity = 200;
  delta = 0;
  IR_kp = 120;
  IR_kd = 60;
  IR_velocity = 450;
  armSpeed = 550;
  beginIR = 250;
  endIR = 2000;
  ramping = 0;
  count = 0;
  turnSpeed = 700;
  turnCount = 4;
  returnIRStart = 200;
  returnIRStop = 1300;
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


