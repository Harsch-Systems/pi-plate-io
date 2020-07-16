#include <stdbool.h>
#include <stdarg.h>

#define DAQC 8
#define MOTOR 16
#define RELAY 24
#define DAQC2 32
#define THERMO 40
#define TINKER 48

struct piplate {
	char id;
	char addr;
	char mapped_addr;
	bool isValid;
	bool ack;
};

extern struct piplate	pi_plate_init(char, char);

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

extern void	relayON(struct piplate*, char);
extern void	relayOFF(struct piplate*, char);
extern void	relayTOGGLE(struct piplate*, char);
extern void	relayALL(struct piplate*, char);
extern int	relaySTATE(struct piplate*, char);

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

extern int	CalGetByte(struct piplate*, char);//DAQC2, THERMO
extern void	CalPutByte(struct piplate*, char);
extern void	CalEraseBlock(struct piplate*);

extern void	startOSC(struct piplate*);
extern void	stopOSC(struct piplate*);
extern void	setOSCchannel(struct piplate*, char, char);
extern void	setOSCsweep(struct piplate*, char);
extern void	getOSCtraces(struct piplate*);
extern void	setOSCtrigger(struct piplate*, char, char*, char*, int);
extern void	trigOSCnow(struct piplate*);
extern void	runOSC(struct piplate*);

