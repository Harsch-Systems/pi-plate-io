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
