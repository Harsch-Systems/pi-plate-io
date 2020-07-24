#ifndef PLATEIO_H_INCLUDED
#define PLATEIO_H_INCLUDED

#include <stdbool.h>
#include <stdarg.h>

#define DAQC 8
#define MOTOR 16
#define RELAY 24
#define DAQC2 32
#define THERMO 40
#define TINKER 48

#define CW 0
#define CCW 1

#define FULL_STEP 0
#define HALF_STEP 1
#define QUARTER_STEP 2
#define EIGHTH_STEP 3

#define CELSIUS 'c'
#define FAHRENHEIT 'f'
#define KELVINS 'k'

#define SINE 1
#define TRIANGLE 2
#define SQUARE 3
#define SAWTOOTH_RISING 4
#define SAWTOOTH_FALLING 5
#define NOISE 6
#define SINC 7

#define CM 'c'
#define IN 'i'

struct oscilloscope {
	bool c1State;
	bool c2State;
	char sRate;
	int trace1[1024];
	int trace2[1024];
	char triggerChan;
	char triggerType;
	char triggerEdge;
	int triggerLevel;
};

struct stepperMotorParams {
	char dir;
	char resolution;
	int rate;
	double acc;
};

struct dcMotorParams {
	char dir;
	char speed;
	double acc;
};

struct tempParams {
	char scale[12];
	char type[8];
	double calScale[8];
	double calOffset[8];
	double calBias;
};

struct servoParams {
	double servoLow;
	double servoHigh;
};

struct DAQC2CalParams {
	double calScale[8];
	double calOffset[8];
	double calDAC[8];
	int pwm[2];
};

struct piplate {
	char id;
	char addr;
	char mapped_addr;
	bool isValid;
	bool ack;
	struct oscilloscope* osc;
	struct stepperMotorParams* stm;
	struct dcMotorParams* dc;
	struct tempParams* tmp;
	struct servoParams* servo;
	struct DAQC2CalParams* daqc2p;
};

extern struct piplate	pi_plate_init(char, char);
extern bool	getINT(void);

/* Start of system level functions */

extern int	getADDR(struct piplate*);//Any plate
extern char*	getID(struct piplate*);//Any plate
extern int	getHWrev(struct piplate*);//Any plate
extern int	getFWrev(struct piplate*);//Any plate
extern void	intEnable(struct piplate*);//THERMO, DAQC, DAQC2, MOTOR
extern void	intDisable(struct piplate*);//THERMO, DAQC, DAQC2, MOTOR
extern int	getINTflags(struct piplate*);//THERMO, DAQc, DAQC2
extern int	getINTflag0(struct piplate*);//MOTOR
extern int	getINTflag1(struct piplate*);//MOTOR
extern void	reset(struct piplate*);//Any plate

/* End of system level functions */

/* Start of LED commands */

extern void	setLEDcolor(struct piplate*, char*);
extern void	setLED(struct piplate*);
extern void	clrLEDcolor(struct piplate*, char*);
extern void	clrLED(struct piplate*);
extern void	toggleLEDcolor(struct piplate*, char*);
extern void	toggleLED(struct piplate*);
extern char	getLEDcolor(struct piplate*, char*);
extern char	getLED(struct piplate*);

/* End of LED commands */

/*Start of relay functions */

extern void	relayON(struct piplate*, char);// |RELAY ---relay #, 1-7--- |  |TINKER: ---relay #, 1-2--- |
extern void	relayOFF(struct piplate*, char);// same
extern void	relayTOGGLE(struct piplate*, char);// same
extern void	relayALL(struct piplate*, char);// |RELAY ---relay vals, 0-127--- | |TINKER: ---relay vals, 0-3--- |
extern int	relaySTATE(struct piplate*, char);// |RELAY ---relay #, 1-7--- | |TINKER: ---relay #, 1-2--- |

/* End of relay functions */

/* Start of digitial io functions */

extern void	setMODE(struct piplate*, char, char*);

extern void	setDOUTbit(struct piplate*, char);
extern void	clrDOUTbit(struct piplate*, char);
extern void	toggleDOUTbit(struct piplate*, char);
extern void	setDOUTall(struct piplate*, char);
extern int	getDOUTall(struct piplate*, char);

extern int	getDINbit(struct piplate*, char);
extern int	getDINall(struct piplate*);
extern void	enableDINint(struct piplate*, char, char);
extern void	disableDINint(struct piplate*, char);

/* End of digital io functions */

/* Start of calibration / Flash memory functions */

extern int	CalGetByte(struct piplate*, char);//DAQC2, THERMO
extern void	CalPutByte(struct piplate*, char);
extern void	CalEraseBlock(struct piplate*);

/* End of calibration / Flash memory functions */

/* Start of DAQC2 Oscilloscope functions */

extern void	startOSC(struct piplate*);
extern void	stopOSC(struct piplate*);
extern void	setOSCchannel(struct piplate*, bool, bool);
extern void	setOSCsweep(struct piplate*, char);
extern void	getOSCtraces(struct piplate*);
extern void	setOSCtrigger(struct piplate*, char, char*, char*, int);
extern void	trigOSCnow(struct piplate*);
extern void	runOSC(struct piplate*);

/* End of DAQC2 Oscilloscope functions */

/* Start of stepper motor functions */

extern void	stepperENABLE(struct piplate*);
extern void	stepperDISABLE(struct piplate*);
extern void	stepperINTenable(struct piplate*, char);
extern void	stepperINTdisable(struct piplate*, char);

extern void	stepperCONFIG(struct piplate*, char, char, char, int, double);//motor, dir, step size, rate, acc
extern void	stepperDIR(struct piplate*, char, char);//motor, direction
extern void	stepperRATE(struct piplate*, char, int, char);//motor, rate, step size
extern void	stepperACC(struct piplate*, char, double);

extern void	stepperMOVE(struct piplate*, char, int);//motor, steps
extern void	stepperJOG(struct piplate*, char);//motor
extern void	stepperSTOP(struct piplate*, char);//motor
extern void	stepperOFF(struct piplate*, char);

/* End of stepper motor functions */

/* Start of dc motor functions */

extern void	dcCONFIG(struct piplate*, char, char, char, double);
extern void	dcSPEED(struct piplate*, char, char);
extern void	dcDIR(struct piplate*, char, char);
extern void	dcACC(struct piplate*, char, double);

extern void	dcSTART(struct piplate*, char);
extern void	dcSTOP(struct piplate*, char);

/* End of dc motor functions */

/* Start of motor interrupt functions */

extern void	setSENSORint(struct piplate*, char);
extern void	slrSENSORint(struct piplate*, char);

extern void	enablestepSTOPint(struct piplate*, char);
extern void	disablestepSTOPint(struct piplate*, char);
extern void	enablestepSTEADYint(struct piplate*, char);
extern void	disablestepSTEADYint(struct piplate*, char);

extern void	enabledcSTOPint(struct piplate*, char);
extern void	disabledcSTOPint(struct piplate*, char);
extern void	enabledcSTEADYint(struct piplate*, char);
extern void	disabledcSTEADYint(struct piplate*, char);

/* End of motor interrupt functions */

/* Start of temperature functions */

extern void	setSCALE(struct piplate*, char, char);
extern char	getSCALE(struct piplate*, char);
extern void	setTYPE(struct piplate*, char, char);
extern char	getTYPE(struct piplate*, char);

extern double	getTEMP(struct piplate*, char);
extern double	getCOLD(struct piplate*, char);
extern double	getRAW(struct piplate*, char);

extern void	setLINEFREQ(struct piplate*, char);
extern void	setSMOOTH(struct piplate*);
extern void	clrSMOOTH(struct piplate*);

/* End of temperature functions */

/* Start of ADC functions */

extern double	getADC(struct piplate*, char);
extern double*	getADCall(struct piplate*);

/* End of ADC functions */

/* Start of DAC functions */

extern double	getDAC(struct piplate*, char);
extern void	setDAC(struct piplate*, char, double);

/* End of DAC functions */

/* Start of PWM and freq */

extern void	setPWM(struct piplate*, char, int);
extern int	getPWM(struct piplate*, char);

extern double	getFREQ(struct piplate*);

/* End of PWM and freq */

/* Start of function generator commands */

void	fgON(struct piplate*, char);
void	fgOFF(struct piplate*, char);
void	fgFREQ(struct piplate*, char, int);
void	fgTYPE(struct piplate*, char, char);
void	fgLEVEL(struct piplate*, char, char);

/* End of function generator commands */

/* Start of switch commands */

bool	getSWstate(struct piplate*);
void	enableSWint(struct piplate*);
void	disableSWint(struct piplate*);
void	enableSWpower(struct piplate*);
void	disableSWpower(struct piplate*);

/* End of switch commands */

/* Start of motorplate sensor commands */

char	getSENSORS(struct piplate*);
int	getTACHcoarse(struct piplate*, char);
int	getTACHfine(struct piplate*, char);

/* End of motorplate sensor commands */

/* Start of Servo commands */

void	setSERVO(struct piplate*, char, double);//TINKER
void	setSERVO2(struct piplate*, char, double);//TINKER
void	setSERVOlow(struct piplate*, double);//TINKER
void	setSERVOhigh(struct piplate*, double);//TINKER

/* End of Servo commands */

/* Miscellaneous commands */

double	getRANGE(struct piplate*, char, char);//DAQC, TINKER
double	getRANGEfast(struct piplate*, char, char);//TINKER
bool	getMOTION(struct piplate*, char);//TINKER
double	getPOT(struct piplate*, char, double);//TINKER
bool	getBUTTON(struct piplate*, char);//TINKER

/* End of miscellaneous commands */

#endif /* PLATEIO_H_INCLUDED */
