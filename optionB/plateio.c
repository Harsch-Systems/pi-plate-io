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

int safeExtract(char* buf){
	if(buf)
		return buf[0];
	return INVAL_CMD;
}

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
		int i;
		int size = bytesToReturn >= 0 ? bytesToReturn : BUF_SIZE;
		static char r[BUF_SIZE];
		for(i = 0; i < size; i ++){
			r[i] = m.rBuf[i];
		}
		return r;
	}
	return NULL;
}

bool pi_plate_init(struct piplate* plate, char* id, char addr){
	if(isValid(id, addr)){
		plate->id = id;
		plate->addr = addr;
		plate->mapped_addr = getBaseAddr(id) + addr;
		plate->ack = useACK(id);
		plate->isValid = 1;
		if(getADDR(plate) == plate->addr)
			return 0;
	}
	plate->isValid = 0;
	return 1;
}

/* Start of system commands: */

int getADDR(struct piplate* plate){
	if(plate->isValid){
		int resp = safeExtract(sendCMD(plate, 0x00, 0, 0, 1));
		return (resp > 0 ? resp - getBaseAddr(plate->id) : resp);
	}
	return INVAL_CMD;
}

char* getID(struct piplate* plate){
	if(plate->isValid)
		return sendCMD(plate, 0x01, 0, 0, -1);
	return NULL;
}

int getHWrev(struct piplate* plate){
	if(plate->isValid)
		return safeExtract(sendCMD(plate, 0x02, 0, 0, 1));
	return INVAL_CMD;
}

int getFWrev(struct piplate* plate){
	if(plate->isValid)
		return safeExtract(sendCMD(plate, 0x03, 0, 0, 1));
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

int getINTflags(struct piplate* plate){
	if(plate->isValid && strEquals(plate->id, 3, "DAQC", "DAQC2", "THERMO"))
		return safeExtract(sendCMD(plate, 0x06, 0, 0, 1));
	return INVAL_CMD;
}

int getINTflag0(struct piplate* plate){
	if(plate->isValid && strEquals(plate->id, 1, "MOTOR"))
		return safeExtract(sendCMD(plate, 0x06, 0, 0, 1));
	return INVAL_CMD;
}

int getINTflag1(struct piplate* plate){
	if(plate->isValid && strEquals(plate->id, 1, "MOTOR"))
		return safeExtract(sendCMD(plate, 0x07, 0, 0, 1));
	return INVAL_CMD;
}

void reset(struct piplate* plate){
	if(plate->isValid)
		sendCMD(plate, 0x0F, 0, 0, 0);
}

/* End of system commands */

/* Start of relay commands */

void relayON(struct piplate* plate, char relay){
	if(plate->isValid){
		if(strEquals(plate->id, 1, "RELAY") && relay >= 1 && relay <=7){
			sendCMD(plate, 0x10, relay, 0, 0);
		}else if(strEquals(plate->id, 1, "TINKER") && relay >= 1 && relay <= 2){
			sendCMD(plate, 0x10, relay - 1, 0, 0);
		}
	}
}

void relayOFF(struct piplate* plate, char relay){
	if(plate->isValid){
		if(strEquals(plate->id, 1, "RELAY") && relay >= 1 && relay <= 7){
			sendCMD(plate, 0x11, relay, 0, 0);
		}else if(strEquals(plate->id, 1, "TINKER") && relay >=1 && relay <= 2){
			sendCMD(plate, 0x11, relay - 1, 0, 0);
		}
	}
}

void relayTOGGLE(struct piplate* plate, char relay){
	if(plate->isValid){
		if(strEquals(plate->id, 1, "RELAY") && relay >= 1 && relay <= 7){
			sendCMD(plate, 0x12, relay, 0, 0);
		}else if(strEquals(plate->id, 1, "TINKER") && relay >= 1 && relay <= 2){
			sendCMD(plate, 0x12, relay - 1, 0, 0);
		}
	}
}

void relayALL(struct piplate* plate, char relays){
	if(plate->isValid){
		if(strEquals(plate->id, 1, "RELAY")){
			if(relays >= 0 && relays <= 127)
				sendCMD(plate, 0x13, relays, 0, 0);
		}else if(strEquals(plate->id, 1, "TINKER")){
			if(relays >= 0 && relays <= 3)
				sendCMD(plate, 0x13, relays, 0, 0);
		}
	}
}

int relaySTATE(struct piplate* plate, char relay){
	if(plate->isValid){
		if(strEquals(plate->id, 1, "RELAY")){
			if(relay >= 1 && relay <= 7)
				return safeExtract(sendCMD(plate, 0x14, 0, 0, 1));
		}else if(strEquals(plate->id, 1, "TINKER")){
			if(relay >= 1 && relay <= 2)
				return safeExtract(sendCMD(plate, 0x14, relay, 0, 0));
		}
	}
	return INVAL_CMD;
}

/* End of relay commands */

/* Start of digital output commands */

void setDOUTbit(struct piplate* plate, char bit){
	if(plate->isValid){
		if(strEquals(plate->id, 1, "DAQC")){
			if(bit >= 0 && bit <= 6)
				sendCMD(plate, 0x10, bit, 0, 0);
		}else if(strEquals(plate->id, 1, "DAQC2")){
			if(bit >= 0 && bit <= 7)
				sendCMD(plate, 0x10, bit, 0, 0);
		}else if(strEquals(plate->id, 1, "TINKER")){
			if(bit >= 1 && bit <= 8)
				sendCMD(plate, 0x26, bit - 1, 0, 0);
		}
	}
}

void clrDOUTbit(struct piplate* plate, char bit){
	if(plate->isValid){
		if(strEquals(plate->id, 1, "DAQC")){
			if(bit >= 0 && bit <= 6)
				sendCMD(plate, 0x11, bit, 0, 0);
		}else if(strEquals(plate->id, 1, "DAQC2")){
			if(bit >= 0 && bit <= 7)
				sendCMD(plate, 0x11, bit, 0, 0);
		}else if(strEquals(plate->id, 1, "TINKER")){
			if(bit >= 1 && bit <= 8)
				sendCMD(plate, 0x27, bit - 1, 0, 0);
		}
	}
}

void toggleDOUTbit(struct piplate* plate, char bit){
	if(plate->isValid){
		if(strEquals(plate->id, 1, "DAQC")){
			if(bit >= 0 && bit <= 6)
				sendCMD(plate, 0x12, bit, 0, 0);
		}else if(strEquals(plate->id, 1, "DAQC2")){
			if(bit >= 0 && bit <= 7)
				sendCMD(plate, 0x12, bit, 0, 0);
		}else if(strEquals(plate->id, 1, "TINKER")){
			if(bit >= 1 && bit <= 8)
				sendCMD(plate, 0x28, bit - 1, 0, 0);
		}
	}
}

/* End of digital output commands */
