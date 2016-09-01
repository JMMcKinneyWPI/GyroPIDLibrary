#ifndef NERD_Gyro_h
#define NERD_Gyro_h

struct SGyroConfig{
	float m_fStdDev;
	float m_fAvg;
	float m_fVoltsPerDPS;
};

typedef struct {
	struct SGyroConfig m_config;
	float m_fAngle;
	int m_iPortNum;
} Gyro;

#endif
