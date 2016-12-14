#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <iostream>

#define FORCE 0,1,2,3


class ADC {
	
	public:
	int readAnalog(int a); //use 1-4 for Analog inputs
	float readForce(int pin);
	
};
