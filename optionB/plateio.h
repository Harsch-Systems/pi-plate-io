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
	char acc;
};

struct dcMotorParams {
	char dir;
	char speed;
	char acc;
};

struct tempParams {
	char scale[12];
	char type[8];
	double calScale[8];
	double calOffset[8];
	double calBias;
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
};

extern struct piplate	pi_plate_init(char, char);
extern double	binaryToDouble(char*);
extern bool	getINT(void);

/* Start of system level functions */

extern int	getADDR(struct piplate*);
extern char*	getID(struct piplate*);
extern int	getHWrev(struct piplate*);
extern int	getFWrev(struct piplate*);
extern void	intEnable(struct piplate*);//0x04
extern void	intDisable(struct piplate*);//0x05
extern int	getINTflags(struct piplate*);//0x06
extern int	getINTflag0(struct piplate*);
extern int	getINTflag1(struct piplate*);
extern void	reset(struct piplate*);

/* End of system level functions */

/*Start of relay functions */

extern void	relayON(struct piplate*, char);
extern void	relayOFF(struct piplate*, char);
extern void	relayTOGGLE(struct piplate*, char);
extern void	relayALL(struct piplate*, char);
extern int	relaySTATE(struct piplate*, char);

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

extern void	stepperCONFIG(struct piplate*, char, char, char, int, char);//motor, dir, step size, rate, acc
extern void	stepperDIR(struct piplate*, char, char);//motor, direction
extern void	stepperRATE(struct piplate*, char, int, char);//motor, rate, step size
extern void	stepperACC(struct piplate*, char, char);

extern void	stepperMOVE(struct piplate*, char, int);//motor, steps
extern void	stepperJOG(struct piplate*, char);//motor
extern void	stepperSTOP(struct piplate*, char);//motor
extern void	stepperOFF(struct piplate*, char);

/* End of stepper motor functions */

/* Start of dc motor functions */

extern void	dcCONFIG(struct piplate*, char, char, char, char);
extern void	dcSPEED(struct piplate*, char, char);
extern void	dcDIR(struct piplate*, char, char);
extern void	dcACC(struct piplate*, char, char);

extern void	dcSTART(struct piplate*, char);
extern void	dcSTOP(struct piplate*, char);

/* End of dc motor functions */

/* Start of temperature functions */

extern void	setSCALE(struct piplate*, char, char);
extern char	getSCALE(struct piplate*, char);
extern void	setTYPE(struct piplate*, char, char);
extern char	getTYPE(struct piplate*, char);

extern double	getTEMP(struct piplate*, char);
extern double	getCOLD(struct piplate*);
extern double	getRAW(struct piplate*, char);

extern void	setLINEFREQ(struct piplate*, char);
extern void	setSMOOTH(struct piplate*);
extern void	clrSMOOTH(struct piplate*);

/* End of temperature functions */
