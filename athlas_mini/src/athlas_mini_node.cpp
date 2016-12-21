#include "athlas_mini/athlas_mini.hpp"
#include "athlas_mini/adc_mini.hpp"

int main(int argc, char **argv) {
	
	ros::init(argc,argv,"ATHLAS_mini");
	
	ros::start();

	ROS_INFO_STREAM("ATHLASmini started");


	ATHLAS_mini athlas;
	ADC adc;
	athlas.setupGPIO();

	while(ros::ok()) {
		for (int i=0; i<4; i++) 
			ROS_INFO_STREAM(i);
	}	
	athlas.lightLED(18,1);
	sleep(1000);
	athlas.lightLED(18,0);
	ROS_INFO_STREAM("Servo set");

	ros::shutdown();
	return 0;
}
