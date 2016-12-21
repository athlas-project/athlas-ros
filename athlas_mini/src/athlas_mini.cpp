#include "athlas_mini/athlas_mini.hpp"

#define LEG_STATE_UP 0
#define LEG_STATE_FORCE_CONTROL 1
#define LEG_STATE_LOCKED 2

#define SERVO_RIGHT 1
#define SERVO_LEFT  0



ExecutionInterval::ExecutionInterval(unsigned int interval){
    this->interval = interval;
    this->lastExecution = 0;
}

/* 
 * Test if enough time has past since last execution 
 * and if so reset
 * 
 * OUTPUT:
 * true if more time than interval
 */
bool ExecutionInterval::testInterval(){
  unsigned long currentMillis = millis();
    if (currentMillis - lastExecution >= interval){
      lastExecution = currentMillis;
      return true;
    }
    return false;
}



void ATHLAS_mini::setupGPIO() {
	wiringPiSetupSys();
}

void ATHLAS_mini::lightLED(int pin, int state) {
	switch(state) {
		case 0: 
			digitalWrite(pin,LOW);
			break;
		case 1: 
			digitalWrite(pin,HIGH);
			break;
	}
}

void ATHLAS_mini::setupLeg(unsigned int servoPin,unsigned int analogPin,unsigned int servoDir){
  this->pid.setTa(5);
  
  this->pid.setKp(0.003);
  this->pid.setKd(0.00006);
  
  /*servo.attach(servoPin);*/
  this->analogPin = analogPin;
  this->state = LEG_STATE_FORCE_CONTROL;
  this->servoDir = servoDir;
  
  this->height = -90; //start fully extended

  this->forceValueReference = 30;
  
}

void ATHLAS_mini::setHeight(double height){
  bool windup = false;
  int pos = (int)heightToAngle(&height,&windup);
  int pos_micros = 1000;

  
  /*if(servoDir == SERVO_RIGHT){
    //Map to servo pwm range for 60°
    pos_micros = map(pos,802,1853,850,1450); //TODO 
  }else{
    //Map to servo pwm range for 60°
    pos_micros = map(pos,802,1853,1900,1250); //TODO 
  }*/
  

  //pos_micros = map(pos,802,1853,1900,1250); //TODO 

  this->height = height;
  this->pid.setWindup(windup);
  this->servo.writeMicroseconds(pos_micros);
}

void ATHLAS_mini::setForce(int force){
  this->forceValueReference = force;
}

void ATHLAS_mini::updateControllLoop(){
  if(state == LEG_STATE_UP){
    this->setHeight(0);
  }else if(state == LEG_STATE_FORCE_CONTROL){
    int forceNow = (int) (analogRead(this->analogPin)+analogRead(this->analogPin)+analogRead(this->analogPin)+analogRead(this->analogPin))/4;
    if (forceNow == 0){
      forceNow = -70;//for faster extending of the legs
    }
    
    double correction = pid.trackReference(this->forceValueReference,forceNow);
    height += correction;
    this->setHeight(height);
  }else if(state == LEG_STATE_LOCKED){
    //stay on same height.
    //DPRINTLN("locked");
  }
}

int ATHLAS_mini::getForce(){
  return analogRead(this->analogPin);
}

double ATHLAS_mini::getHeight(){
  return this->height;
}

int ATHLAS_mini::state;


/* calculate servo angle to reach height of leg
 *  Masstab 1:5.5
 *  INPUT:
 *  double height [mm] 
 *    0        => 26.58 mm unter Helikopter (höchste Position)
 *    -93.438  => 120 mm unter Helikopter (unterste Position)
 *    
 *   bool windup  , is set to true if height out of boundries (to prevent windup)
 *  
 *  OUTPUT:
 *  double angle [rad]  (45.98° - 106.192°) 0.8026-1.8534
 */

double ATHLAS_mini::heightToAngle(double *height_pointer,bool *windup){
  //correct height if too big or too small

  if (*height_pointer > 0){
    *height_pointer = 0;
    *windup = true;
  }else if(*height_pointer < -93.438){
    *height_pointer = -93.438;
    *windup = true;
  }else{
    *windup = false;
  }

  int height = *height_pointer;

  height -= 26.58;            //set Zero to -26.58mm

  //calculate angle in rad 
  return 503.7-2.0448*height*5.5; //Set scale to 5.5
}
