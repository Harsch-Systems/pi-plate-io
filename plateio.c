#include <stdio.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>

#include "../pi-plate-module/module/piplate.h"
#include "plateio.h"

#define DAQC_BASE_ADDR 8
#define MOTOR_BASE_ADDR 16
#define RELAY_BASE_ADDR 24
#define DAQC2_BASE_ADDR 32
#define THERMO_BASE_ADDR 40
#define TINKER_BASE_ADDR 48

#define INVAL_ID -1

char getBaseAddr(char* id){
	if(!strcmp(id, "DAQC"))
		return DAQC_BASE_ADDR;
	else if(!strcmp(id, "MOTOR"))
		return MOTOR_BASE_ADDR;
	else if(!strcmp(id, "RELAY"))
		return RELAY_BASE_ADDR;
	else if(!strcmp(id, "DAQC2"))
		return DAQC2_BASE_ADDR;
	else if(!strcmp(id, "THERMO"))
		return THERMO_BASE_ADDR;
	else if(!strcmp(id, "TINKER"))
		return TINKER_BASE_ADDR;
	else
		return 0;
}

bool useACK(char* id){
	return (!strcmp(id, "DAQC2") || !strcmp(id, "THERMO") || !strcmp(id, "TINKER"));
}

bool isValid(char* id){
	return (!strcmp(id, "DAQC2") || !strcmp(id, "THERMO") || !strcmp(id, "TINKER") || !strcmp(id, "DAQC") || !strcmp(id, "RELAY") || !strcmp(id, "MOTOR"));
}

static char* sendCMD(unsigned char addr, unsigned char cmd, unsigned char p1, unsigned char p2, int bytesToReturn, char* id){
	struct message m = BASE_MESSAGE;

	FILE *fp = fopen("/dev/PiPlates", "r");

	if(!fp)
		return NULL;

	m.addr = addr + getBaseAddr(id);
	m.cmd = cmd;
	m.p1 = p1;
	m.p2 = p2;
	m.bytesToReturn = bytesToReturn;
	m.useACK = useACK(id);

	ioctl(fileno(fp), PIPLATE_SENDCMD, &m);

	fclose(fp);

	if(bytesToReturn){
		int size = bytesToReturn > 0 ? bytesToReturn : BUF_SIZE;
		static char r[BUF_SIZE];
		for(int i = 0; i < size; i ++){
			r[i] = m.rBuf[i];
		}
		return r;
	}
	return NULL;
}

/* Start of commands: */

char getADDR(char* id, char addr){
	if(isValid(id))
		return sendCMD(addr, 0x00, 0, 0, 1, id)[0];
	return INVAL_ID;
}

char* getID(char* id, char addr){
	if(isValid(id))
		return sendCMD(addr, 0x01, 0, 0, -1, id);
	return NULL;
}

char getHWrev(char* id, char addr){
	if(isValid(id))
		return sendCMD(addr, 0x02, 0, 0, 1, id)[0];
	return INVAL_ID;
}

char getFWrev(char* id, char addr){
	if(isValid(id))
		return sendCMD(addr, 0x03, 0, 0, 1, id)[0];
	return INVAL_ID;
}

void intEnable(char* id, char addr){
	if(!strcmp(id, "DAQC") || !strcmp(id, "DAQC2") || !strcmp(id, "MOTOR") || !strcmp(id, "THERMO")){
		sendCMD(addr, 0x04, 0, 0, 0, id);
	}
}

void intDisable(char* id, char addr){
	if(!strcmp(id, "DAQC") || !strcmp(id, "DAQC2") || !strcmp(id, "MOTOR") || !strcmp(id, "THERMO")){
		sendCMD(addr, 0x05, 0, 0, 0, id);
	}
}

char* getINTflags(char* id, char addr){
	if(!strcmp(id, "DAQC") || !strcmp(id, "DAQC2") || !strcmp(id, "THERMO")){

	}else if(!strcmp(id, "THERMO")){
		static char r[2];
	}
}

/* End of commands */

