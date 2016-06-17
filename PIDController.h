 /* QCC2 PID Controller Library
  * Jason McKinney
  * 07/22/2015
<!--             07/22 Update            -->
	- Fixed bug in slew rate calculation that caused 
		the output value to wind faster than rate should allow. 
*/

#ifndef PIDController_h
#define PIDController_h

typedef struct Controller{
	float timer, dT; //For delta time of controller operation
	float kP, kI, kD, epsilon; //Controller constants
	int pV, setPoint, lastPV; //sensor read (process Variable), loop set point and previous sensorVal
	float errorSum; //Sumnation of error while in deadband
	float output, lastOutput;
}PID;

//initialize the PID controller with desired constants
void pidInit(PID &pid, float kP, float kI, float kD, float epsilon){
	pid.timer = nPgmTime;
	pid.kP = kP;
	pid.kI = kI;
	pid.kD = kD;
	pid.epsilon = epsilon;
	pid.error = 0;
	pid.pV = 0;
	pid.output = 0;
	pid.lastOutput = 0;
}

//Calculate and filter the control loop output for use with vex motors
float pidFilteredOutput(PID &pid){
	float filteredOut = pid.output;
	if(pid.dT != 0)
		filteredOut = pid.output;
	if(abs(filteredOut) > 127)
		filteredOut = 127 * filteredOut/abs(filteredOut);

	pid.lastOutput = filteredOut;
	return filteredOut;`
}

void pidSet(PID &pid, int setPoint){
	pid.setPoint = setPoint;
}

//Execute the control loop and return its output
float pidExecute(PID &pid, int sensorValue, int setPoint){
	pidSet(pid, setPoint);
	pid.lastPV = pid.pV;
	pid.pV = sensorValue;
	pid.error = pid.setPoint - pid.pV;

	pid.dT = (nPgmTime - pid.timer)/1000; //delta time in seconds
	pid.timer = nPgmTime;
	delay(10);

	float rate;
	if(abs(pid.dT) > 0)
		rate = (pid.pV - pid.lastPV)/pid.dT;
	else
		rate = 0;

	if(abs(error) > pid.epsilon)
		pid.errorSum += error*pid.dT;

	pid.output = error * pid.kP +
								 pid.errorSum * pid.kI +
								 rate * pid.kD;

	return pidFilteredOutput(pid);
}
#endif
