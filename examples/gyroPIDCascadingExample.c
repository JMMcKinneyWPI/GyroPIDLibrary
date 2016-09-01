#pragma platform(VEX)

#pragma competitionControl(Competition)
#pragma autonomousDuration(20)
#pragma userControlDuration(120)

#include "Vex_Competition_Includes.c"
#include "../libGyro/NERD_Gyro.c"
#include "../libPID/NERD_PID.c"

#define SET_ANGLE 90

Gyro gyro;
PID gyroRatePid; //inner PID loop
PID gyroAnglePid; //outer PID loop

//power left drive motors
void
driveL(int val){
	val = abs(val) > 127 ? 127 * val/abs(val) : val;
	motor[port2] = val;
}

//power right drive motors
void
driveR(int val){
	val = abs(val) > 127 ? 127 * val/abs(val) : val;
	motor[port3] = val;
}

//gyro turn to target angle
void
gyroTurn(float fTarget){
	bool bAtGyro = false;
	long liAtTargetTime = nPgmTime;
	long liTimer = nPgmTime;
	float fGyroAngle = 0;

	while(!bAtGyro){
		//Calculate the delta time from the last iteration of the loop
		float fDeltaTime = (float)(nPgmTime - liTimer)/1000.0;
		//Reset loop timer
		liTimer = nPgmTime;

		float fGyroRate = gyroGetRate(gyro);
		fGyroAngle +=  fGyroRate * fDeltaTime;

		//					  -------------------------------<-
		//					 |				|
		//				 fgyroAngle 	fgyroRate 			 	Physical System
		//					 v				v
		// fTarget -> gyroAnglePID -> gyroRatePID -> motors--->
		//
		float driveOut = pidCalculate(gyroRatePid, pidCalculate(gyroAnglePid, fTarget, fGyroAngle), fGyroRate);

		driveL(-driveOut);
		driveR(driveOut);

		//Stop the turn function when the angle has been within 3 degrees of the desired angle for 350ms
		if(abs(fTarget - fGyroAngle) > 3)
			liAtTargetTime = nPgmTime;
		if(nPgmTime - liAtTargetTime > 350){
			bAtGyro = true;
			driveL(0);
			driveR(0);
		}
	}
}

//Calibrate gyro and initialize PID controllers
void
pre_auton(){
	//Allow gyro to settle and then init/calibrate (Takes a total of around 2 seconds)
	delay(1100);
	gyroInit(gyro, 1);

	//These NEED to be tuned
	pidInit(gyroAnglePid, 2, 0, 0.15, 2, 20.0); //No idea if these are any good, they need to be tuned a TON
	pidInit(gyroRatePid, 10, 0, 0, 0, 360.00); //need to be tuned
}

task
autonomous(){
	//Turn the robot by the desired angle
	gyroTurn(SET_ANGLE);
}

task
usercontrol(){
}
