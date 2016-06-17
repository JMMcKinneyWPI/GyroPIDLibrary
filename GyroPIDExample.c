/* Gyro and PID example code
 * by Jason McKinney from QCC2
 * 4/20/2015
*/

#pragma config(Sensor, dgtl1, calibrationInProgress,        sensorLEDtoVCC)

#pragma platform(VEX)

//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(20)
#pragma userControlDuration(120)

#include "Vex_Competition_Includes.c"   //Main competition background code...do not modify!
#include "gyro.c"
#include "PIDController.h"

PID gyroPid;
float setAngle = 90;

//Power left drive motors
void driveL(int val)
{
	if(abs(val) > 127)
		val = 127 * val/abs(val);

	motor[port2] = val;
}

//Power right drive motors
void driveR(int val)
{
	if(abs(val) > 127)
		val = 127 * val/abs(val);
	
	motor[port3] = val;
}

//Gyro turn to target angle
void gyroTurn(float target)
{
	if(abs(target) < 40)
		pidInit(gyroPid, 3, 0, 0.15, 3);
	bool atGyro = false;
	long atTargetTime = nPgmTime;
	long timer = nPgmTime;

	pidReset(gyroPid);
	gyroResetAngle();
	while(!atGyro)
	{
		//Calculate the delta time from the last iteration of the loop
		float dT = (float)(nPgmTime - timer)/1000;
		
		//Calculate the current angle of the gyro
		float angle = gyroAddAngle(dT);

		//Reset loop timer
		timer = nPgmTime;
		
		//Calculate the output of the PID controller and output to drive motors
		float driveOut = pidExecute(gyroPid, angle, target);
		driveL(-driveOut);
		driveR(driveOut);

		//Stop the turn function when the angle has been within 3 degrees of the desired angle for 350ms
		if(abs(error) > 3)
			atTargetTime = nPgmTime;
		if(nPgmTime - atTargetTime > 350)
		{
			atGyro = true;
			driveL(0);
			driveR(0);
		}
	}
	
	//Reinitialize the PID constants to their original values in case they were changed
	pidInit(gyroPid, 2, 0, 0.15, 2);
}

//Calibrate gyro and initialize PID controller
void pre_auton()
{
	//Set gyro port to analog port 1
	gyroSetPort(in1);
	
	//Allow gyro to settle and then calibrate (Takes a total of around 3 seconds)
	delay(1100);
	SensorValue[calibrationInProgress] = 1;
	gyroCalibrate();
	SensorValue[calibrationInProgress] = 0;
	
	/*Initialize PID controller for gyro
	 * kP = 2, kI = 0, kD = 0.15
	 * epsilon = 0
	*/
	pidInit(gyroPid, 2, 0, 0.15, 0);
}

task autonomous()
{
	//Turn the robot by the desired angle
	gyroTurn(setAngle);
}

task usercontrol()
{
}