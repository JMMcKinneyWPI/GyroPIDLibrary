#ifndef NERD_PID_h
#define NERD_PID_h

typedef struct{
	float m_fKP;
	float m_fKI;
	float m_fKD;
	float m_fEpsilonInner;
	float m_fEpsilonOuter;
	float m_fSigma;
	float m_fLastValue;
	unsigned long m_uliLastTime;
	float m_fLastSetPoint;
} PID;

#endif
