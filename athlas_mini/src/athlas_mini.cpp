#include "athlas_mini/athlas_mini.hpp"

void ATHLAS_mini::setupGPIO() {
	wiringPiSetupSys();
}

void ATHLAS_mini::setServo(int pwm, int pin) {
	digitalWrite(pin,LOW);
	softPwmCreate(pin,0,200);
	softPwmWrite(pin,pwm);
}

double ATHLAS_mini::senseGround() {

}
