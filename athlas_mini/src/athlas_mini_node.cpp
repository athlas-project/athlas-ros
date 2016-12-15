#include "athlas_mini/athlas_mini.hpp"
#include "athlas_mini/adc_mini.hpp"

int main(int argc, char **argv) {
	
	ros::init(argc,argv,"ATHLAS-mini started");
	
	ros::start();

	ROS_INFO_STREAM("ATHLAS-mini started");


	ATHLAS_mini athlas;
	ADC adc;
	athlas.setupGPIO();

	int force_pins [4] = {FORCE};
	double force [4] = {};
	
	while(ros::ok()) {
		for (int i=0; i<4; i++) 
			force[i] = adc.readForce(force_pins[i]);
	}

	ros::shutdown();
	return 0;
}
