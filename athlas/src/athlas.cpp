#include "athlas/athlas.hpp"

void Athlas::setupGPIO() {
	wiringPiSetupSys();
}

void Athlas::setServo(int pwm, int pin) {
	digitalWrite(pin,LOW);
	softPwmCreate(pin,0,200);
	softPwmWrite(pin,pwm);
}

bool Athlas::senseGround() {

}
