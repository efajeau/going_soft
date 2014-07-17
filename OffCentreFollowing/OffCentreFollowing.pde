/**
 * Tape following algorithm for an off-centre follow
 */

#include <phys253.h>
#include <LiquidCrystal.h>
#include <Servo253.h>
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
int correction = 5;
int delta = 0.0;
int LEFT_QRD_INPUT = 3;
int RIGHT_QRD_INPUT = 4;

void setup() {
  Serial.begin(9600);
  pinMode(0, INPUT);
}

void loop() {
  //MENU TO SET PID
   while ( !(stopbutton()) ){
   
    kp = knob(6);
    kd = knob(7);
    LCD.home() ;
    LCD.setCursor(0,0); LCD.print("P: "); LCD.print(kp);
    LCD.setCursor(0,1);LCD.print("D: "); LCD.print(kd);
    delay(10);
    LCD.clear(); 
  }
  //MENU TO SET THRESHOLD FOR QRD
  while ( !(startbutton()) ){
    
    int left = analogRead(LEFT_QRD_INPUT);
    int right = analogRead(RIGHT_QRD_INPUT);
    threshold = knob(6);
    LCD.setCursor(0,1); LCD.print("Threshold: ");LCD.print(threshold);
    LCD.setCursor(0,0); LCD.print("L: ");LCD.print(left); LCD.print(" R: ");LCD.print(right);
    delay(100);
    LCD.clear();
  
  }
  
  while (startbutton() ){}
  //MENU TO SET SPEED
  while( !(startbutton()) ){
    velocity = knob(6);
    delta = floor(knob(7)/1023.0*5.0);
    if( velocity > 700 ){
      velocity = 700;
    }
    
    LCD.setCursor(0,0); LCD.print("SPEED: "); LCD.print(velocity);
    LCD.setCursor(0,1); LCD.print("DELTA: "); LCD.print(delta);
    delay(10);
    LCD.clear();
  }
  //TAPE FOLLOWING ALGORITHM
  while ( !(stopbutton()) ) {

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
    i = ki*error+i;
    pd = p + d;
    
    motor.speed(0,velocity+pd);
    motor.speed(1,velocity-pd);
    time = time + 1;
  }
  while(stopbutton())
  { 
    motor.stop(0);
    motor.stop(1);
  }
}

