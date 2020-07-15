#include <stdbool.h>
#include <stdarg.h>

struct piplate {
	char* id;
	char addr;
	char mapped_addr;
	bool isValid;
	bool ack;
};

extern bool	pi_plate_init(struct piplate*, char*, char);
extern char	getADDR(struct piplate*);
extern char*	getID(struct piplate*);
extern char	getHWrev(struct piplate*);
extern char	getFWrev(struct piplate*);
extern void	intEnable(struct piplate*);//0x04
extern void	intDisable(struct piplate*);//0x05
extern char	getINTflags(struct piplate*);//0x06
extern char	getINTflag0(struct piplate*);
extern char	getINTflag1(struct piplate*);
extern void	reset(struct piplate*);
/*
extern void	relayON(char*, char, char);
extern void	relayOFF(char*, char, char);
extern void	relayTOGGLE(char*, char, char);
extern void	relayALL(char*, char, char);
extern char	relaySTATE(char*, char);
extern void	setDOUTbit(char*, char, char);
extern void	clrDOUTbit(char*, char, char);
extern void	toggleDOUTbit(char*, char, char);
extern void	setDOUTall(char*, char, char);
extern char	getDOUTbyte(char*, char);
*/
