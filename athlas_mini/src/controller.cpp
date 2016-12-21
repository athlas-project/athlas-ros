//generate controller object
#include "Arduino.h"
#include "controller.h"

#define DPRINT(...)    //Serial.print(__VA_ARGS__)     
#define DPRINTLN(...)  //Serial.println(__VA_ARGS__) 

Controller::Controller(double p,double i,double d,int ta) {
      Kp = p;
      Ki = i;
      Kd = d;
      Ta = ta;
      
      i_sum = 0;
      e_old = 0;
      antiWindup = false;
}

Controller::Controller(){
  i_sum = 0;
  e_old = 0;
  antiWindup = false;
}

void Controller::setTa(int ta){
  Ta = ta;
}

void Controller::setKp(double p){
  Kp = p;
}

void Controller::setKi(double i){
  Ki = i;
  i_sum = 0;
}

void Controller::setKd(double d){
  Kd = d;
  e_old = 0;
}

/*
 * Change Windup state
 * INPUT
 * bool windup , true if actuator limited
 */
void Controller::setWindup(bool windup){
  antiWindup = windup;
}
    
/* Calculate pid control
 * 
 * INPUT:
 * double r: reference value
 * double x: system state to track reference value
 * 
 * OUTPUT:
 * double System input to track reference value
 */
double Controller::trackReference(double r, double x) {
  double e = x-r;
  DPRINT("reference: ");
  DPRINT(r);
  DPRINT(" measurement: ");
  DPRINT(x);
  
  //if actuator is in its limits stop integrating
  if(!antiWindup){
    i_sum = i_sum + e; 
  }
  
  double u = Kp * e + Ki * Ta/1000 * i_sum + Kd * ((e-e_old)*1000)/Ta; 
  e_old = e; 

  DPRINT(" output: ");
  DPRINTLN(u);
  return u; 
}


