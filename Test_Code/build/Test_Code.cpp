/**
 * DigitalReadSerial
 * Reads a digital input pin 0, prints the result to the serial monitor 
 */

#include <phys253.h>          //   ***** from 253 template file
#include <LiquidCrystal.h>    //   ***** from 253 template file
#include <Servo253.h>         //   ***** from 253 template file


#include "WProgram.h"
#include <HardwareSerial.h>
void setup();
void loop();
void tuneParameters();
void tapeFollow ();
void swingArm();
int leftMotor = 0;
int rightMotor = 1;
int armMotor = 2;

int startSwitchPin= 0;
int endSwitchPin = 1;
int contactSwitchPin = 2;

int error;
int last_error;
int stored_lerr;
int kp;
int kd;
int ki;
int p;
int d;
int i;
int pd;
int time;
int store_time;
int threshold;
int velocity; 

void setup() {
  
  portMode(0, INPUT) ;      	 	//***** from 253 template file
  portMode(1, INPUT) ;      	   	//***** from 253 template file
  RCServo0.attach(RCServo0Output) ;	//***** from 253 template file
  RCServo1.attach(RCServo1Output) ;	//***** from 253 template file
  RCServo2.attach(RCServo2Output) ;	//***** from 253 template file
  
  Serial.begin(9600);
  
  pinMode(startSwitchPin, INPUT);
  pinMode(endSwitchPin, INPUT);
  pinMode(contactSwitchPin, INPUT);
  
  digitalWrite(contactSwitchPin, HIGH);
  
  //attachInterrupt(contactSwitchPin, swingArm, RISING);
}

void loop() {
    
    while( !(startbutton()) && !(stopbutton()) ) {
      LCD.clear();  
      LCD.home();
      LCD.setCursor(0,0);
      LCD.print("HOLD STOP");
      LCD.setCursor(0,1);
      LCD.print("TO TUNE");
      
      delay(1000);
      
      LCD.clear();  
      LCD.home();
      LCD.setCursor(0,0);
      LCD.print("HOLD START");
      LCD.setCursor(0,1);
      LCD.print("TO RUN");
      
      delay(1000);
    }
    
    //GOES TO TUNE PARAMETERS FUNCTION
    while( stopbutton() ) {
      tuneParameters();
    }
    
    //RUNS MAIN CODE WHILE
    //PRESS STOP BUTTON TO STOP
    while( startbutton() ){
      while( !(stopbutton()) ){
          digitalWrite(contactSwitchPin, HIGH);
          
          if( digitalRead(contactSwitchPin) == HIGH){
            swingArm();
          }
          
          int velocity  = 2*knob(6) - 1023;
          
          motor.speed(leftMotor, velocity);
          motor.speed(rightMotor, velocity);
            
          LCD.clear();  
          LCD.home();
          LCD.setCursor(0,0); 
          LCD.print("Main Code");
          delay(100);
        
      }
    
    }
    while( stopbutton() ){
      motor.stop_all();  
    }
    
}

void tuneParameters(){
      //MENU TO SET KP AND KD VALUES
      while( !(startbutton()) ) {
         kp = knob(6);
         kd = knob(7);
         LCD.home() ;
         LCD.setCursor(0,0); LCD.print("P: "); LCD.print(kp);
      LCD.setCursor(0,1);LCD.print("D: "); LCD.print(kd);
      delay(10);
      LCD.clear();  
      }
      
      while( startbutton() ) { }
      
      //MENU TO SET VELOCITY
      while( !(startbutton()) ){
        velocity = knob(6);
        if( velocity > 700 ){
          velocity = 700;
        }
        LCD.setCursor(0,0); LCD.print("SPEED: ");LCD.print(velocity);
        delay(10);
        LCD.clear(); 
      }
}

void tapeFollow (){

}


// Swings arm one full rotation
void swingArm() {
     LCD.clear();  
     LCD.home();
     LCD.setCursor(0,0); LCD.print("Swinging");
     LCD.setCursor(0,1); LCD.print("Back");
     
  while( digitalRead(endSwitchPin) != HIGH ){
     motor.speed(armMotor, -300);
     digitalWrite(endSwitchPin, HIGH);
  }
  
  LCD.clear();
  motor.stop(armMotor);

     LCD.home();
     LCD.setCursor(0,0); LCD.print("Swinging Arm");
     LCD.setCursor(0,1); LCD.print("Forward");
     
  while( digitalRead(startSwitchPin) != HIGH ){
     motor.speed(armMotor, 300);
     digitalWrite(startSwitchPin, HIGH);
  }
  
  LCD.clear();
  
  motor.stop(armMotor);

}



