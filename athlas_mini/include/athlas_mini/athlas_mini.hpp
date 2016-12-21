
#include <ros/ros.h>
#include <wiringPi.h>
#include <softPwm.h>
#include "controller.hpp"

//Pin definition for GPIO, use BCM number
//
//+-----+-----+---------+------+---+---Pi 3---+---+------+---------+-----+-----+
//| BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
//+-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
//|     |     |    3.3v |      |   |  1 || 2  |   |      | 5v      |     |     |
//|   2 |   8 |   SDA.1 | ALT0 | 1 |  3 || 4  |   |      | 5V      |     |     |
//|   3 |   9 |   SCL.1 | ALT0 | 1 |  5 || 6  |   |      | 0v      |     |     |
//|   4 |   7 | GPIO. 7 |   IN | 1 |  7 || 8  | 1 | ALT5 | TxD     | 15  | 14  |
//|     |     |      0v |      |   |  9 || 10 | 1 | ALT5 | RxD     | 16  | 15  |
//|  17 |   0 | GPIO. 0 |   IN | 0 | 11 || 12 | 0 | ALT0 | GPIO. 1 | 1   | 18  |
//|  27 |   2 | GPIO. 2 |   IN | 0 | 13 || 14 |   |      | 0v      |     |     |
//|  22 |   3 | GPIO. 3 |   IN | 0 | 15 || 16 | 0 | IN   | GPIO. 4 | 4   | 23  |
//|     |     |    3.3v |      |   | 17 || 18 | 0 | IN   | GPIO. 5 | 5   | 24  |
//|  10 |  12 |    MOSI | ALT0 | 0 | 19 || 20 |   |      | 0v      |     |     |
//|   9 |  13 |    MISO | ALT0 | 0 | 21 || 22 | 0 | IN   | GPIO. 6 | 6   | 25  |
//|  11 |  14 |    SCLK | ALT0 | 0 | 23 || 24 | 1 | OUT  | CE0     | 10  | 8   |
//|     |     |      0v |      |   | 25 || 26 | 0 | IN   | CE1     | 11  | 7   |
//|   0 |  30 |   SDA.0 |   IN | 1 | 27 || 28 | 1 | IN   | SCL.0   | 31  | 1   |
//|   5 |  21 | GPIO.21 |   IN | 1 | 29 || 30 |   |      | 0v      |     |     |
//|   6 |  22 | GPIO.22 |   IN | 1 | 31 || 32 | 0 | IN   | GPIO.26 | 26  | 12  |
//|  13 |  23 | GPIO.23 |   IN | 0 | 33 || 34 |   |      | 0v      |     |     |
//|  19 |  24 | GPIO.24 |   IN | 0 | 35 || 36 | 0 | IN   | GPIO.27 | 27  | 16  |
//|  26 |  25 | GPIO.25 |   IN | 0 | 37 || 38 | 0 | IN   | GPIO.28 | 28  | 20  |
//|     |     |      0v |      |   | 39 || 40 | 0 | IN   | GPIO.29 | 29  | 21  |
//+-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
//| BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
//+-----+-----+---------+------+---+---Pi 3---+---+------+---------+-----+-----+

#define SERVOS 12,14,16,18

class ExecutionInterval{
  public:
    ExecutionInterval(unsigned int interval);
    bool testInterval();

  private:
    unsigned long lastExecution;
    unsigned int interval;
};

class ATHLAS_mini {
	public:
	
	// sets up the GPIO pins of the raspberry pi		
	void setupGPIO();

	// turns LED at 'pin' on/off
	void lightLED(int pin, int state);

    //Attaches servo and sensor to corresponding pins
    void setupLeg(unsigned int servoPin, unsigned int forcePin, unsigned int servoDir);

    //moves leg to corresponding height
    //deactivates force controll
    //height [mm]
    void setHeight(double height);

    //Set reference Value for Force controller 
    //force [0-1024]
    void setForce(int force);

    //Activate Force Controll
    //void activateForceControll();

    //Updates Controll-Loop
    void updateControllLoop();

    //Returns position in rad
    int getPosition();

    //Returns force sensor value
    int getForce();

    //Returns height in mm
    double getHeight();

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
    static double heightToAngle(double *height_pointer,bool *windup);

    static int state;

    Controller pid;

    private:
      double height;
      int position;
      int forceValueReference;
      int analogPin;
      unsigned int servoDir;
};


