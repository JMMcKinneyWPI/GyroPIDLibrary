/* Jason McKinney
 * Gyro Sensor library for Vex Gyroscope
 * 04/12/2015
*/

struct Gyro{
	float angle, gyroOffset, gyroRead, rateOffset;
	int sensorPort;
};
Gyro gyro;


float gyroGetRate()
{
	float scaleFactor = 1.511;
	
	// 1.1 mV/dps
	float sensitivity = 0.0011;
	
	//Voltage from gyro sensor is proportional to sensor read (0 to 4095) multiplied by a scale factor constant 
	float gyroVoltage = (float)SensorValue(gyro.sensorPort) * (5/4095) / scaleFactor;
	
	//Degrees per second = voltage / (volts/degrees per second) * constant
	float rate = gyroVoltage/sensitivity;
	return rate;
}

//Return the calibrated and filtered rate of change that the gyro reads.
//if the gyro reads 2 degrees per second or less, assume that it is noise and return 0
float gyroGetFilteredRate()
{
	float rate = gyroGetRate() - gyro.gyroOffset - gyro.rateOffset;
	if(abs(rate) < 2)
		return 0;
	return rate;
}

//sample and offset the rate of change that the gyro detects.
void gyroCalibrate()
{
	for(int i = 0; i < 10000; i++)
	{
		gyro.gyroOffset += gyroGetRate();
	}
	gyro.gyroOffset /= 10000;

	for(int i = 0; i < 10000; i++)
	{
		gyro.rateOffset += gyroGetRate()- gyro.gyroOffset;
	}
	gyro.rateOffset /= 10000;
}

void gyroSetPort(int sensorPort)
{
	gyro.sensorPort = sensorPort;
}

float gyroAddAngle(float dt)
{
	gyro.angle += gyroGetFilteredRate() * dt;
	return gyro.angle;
}

void gyroResetAngle()
{
	gyro.angle = 0;
}
