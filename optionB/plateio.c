#include <stdio.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>

#include "../../pi-plate-module/module/piplate.h"
#include "plateio.h"

#define DAQC_BASE_ADDR 8
#define MOTOR_BASE_ADDR 16
#define RELAY_BASE_ADDR 24
#define DAQC2_BASE_ADDR 32
#define THERMO_BASE_ADDR 40
#define TINKER_BASE_ADDR 48

#define INVAL_CMD -1

static bool strEquals(char*, int, ...);

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

static bool useACK(char* id){
	return strEquals(id, 3, "DAQ2", "THERMO", "TINKER");
}

static bool isValid(char* id, char addr){
	return (addr >= 0 && addr <= 7) && strEquals(id, 6, "DAQC2", "THERMO", "TINKER", "DAQC", "RELAY", "MOTOR");
}

static bool strEquals(char* id, int num, ...){
	va_list strings;
	int i;

	va_start(strings, num);

	for(i = 0; i < num; i++){
		char* s = va_arg(strings, char*);
		if(!strcmp(id, s)){
			va_end(strings);
			return 1;
		}
	}
	va_end(strings);
	return 0;
}

static char* sendCMD(struct piplate* plate, unsigned char cmd, unsigned char p1, unsigned char p2, int bytesToReturn){
	struct message m = BASE_MESSAGE;

	FILE *fp = fopen("/dev/PiPlates", "r");

	if(!fp)
		return NULL;

	m.addr = plate->mapped_addr;
	m.cmd = cmd;
	m.p1 = p1;
	m.p2 = p2;
	m.bytesToReturn = bytesToReturn;
	m.useACK = plate->ack;
	m.state = 0;

	ioctl(fileno(fp), PIPLATE_SENDCMD, &m);

	fclose(fp);

	if(m.state){
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
	printf("An Error Occured\n");
	return NULL;
}

bool pi_plate_init(struct piplate* plate, char* id, char addr){
	if(isValid(id, addr)){
		plate->id = id;
		plate->addr = addr;
		plate->mapped_addr = getBaseAddr(id) + addr;
		plate->ack = useACK(id);
		if(sendCMD(plate, 0, 0, 0, 1)[0] == plate->mapped_addr){
			plate->isValid = 1;
			return 0;
		}
	}
	plate->isValid = 0;
	return 1;
}

/* Start of commands: */

char getADDR(struct piplate* plate){
	if(plate->isValid)
		return sendCMD(plate, 0x00, 0, 0, 1)[0];
	return INVAL_CMD;
}

char* getID(struct piplate* plate){
	if(plate->isValid)
		return sendCMD(plate, 0x01, 0, 0, -1);
	return NULL;
}

char getHWrev(struct piplate* plate){
	if(plate->isValid)
		return sendCMD(plate, 0x02, 0, 0, 1)[0];
	return INVAL_CMD;
}

char getFWrev(struct piplate* plate){
	if(plate->isValid)
		return sendCMD(plate, 0x03, 0, 0, 1)[0];
	return INVAL_CMD;
}

void intEnable(struct piplate* plate){
	if(plate->isValid && strEquals(plate->id, 4, "DAQC", "DAQC2", "MOTOR", "THERMO"))
		sendCMD(plate, 0x04, 0, 0, 0);
}

void intDisable(struct piplate* plate){
	if(plate->isValid && strEquals(plate->id, 4, "DAQC", "DAQC2", "MOTOR", "THERMO"))
		sendCMD(plate, 0x05, 0, 0, 0);
}

char getINTflags(struct piplate* plate){
	if(plate->isValid && strEquals(plate->id, 3, "DAQC", "DAQC2", "THERMO"))
		return sendCMD(plate, 0x06, 0, 0, 1)[0];
	return INVAL_CMD;
}

char getINTflag0(struct piplate* plate){
	if(plate->isValid && strEquals(plate->id, 1, "MOTOR"))
		return sendCMD(plate, 0x06, 0, 0, 1)[0];
	return INVAL_CMD;
}

char getINTflag1(struct piplate* plate){
	if(plate->isValid && strEquals(plate->id, 1, "MOTOR"))
		return sendCMD(plate, 0x07, 0, 0, 1)[0];
	return INVAL_CMD;
}

void reset(struct piplate* plate){
	if(plate->isValid)
		sendCMD(plate, 0x0F, 0, 0, 0)[0];
}

/* End of commands */

