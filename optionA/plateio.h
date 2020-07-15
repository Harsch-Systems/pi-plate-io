extern char	getADDR(char*, char);
extern char*	getID(char*, char);
extern char	getHWrev(char*, char);
extern char	getFWrev(char*, char);
extern void	intEnable(char*, char);//0x04
extern void	intDisable(char*, char);//0x05
extern char	getINTflags(char*, char);//0x06
extern char	getINTflag0(char*, char);
extern char	getINTflag1(char*, char);
extern void	reset(char*, char);
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
