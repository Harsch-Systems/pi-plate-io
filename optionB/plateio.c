#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <math.h>

#include "../../pi-plate-module/module/piplate.h"
#include "plateio.h"

#define INVAL_CMD -1

const char* modes[9] = {"din", "dout", "button", "pwm", "range", "temp", "servo", "rgbled", "motion"};
const bool pcaRequired[9] = {0, 0, 0, 1, 0, 0, 0, 0, 0};

const double kVoltageCoefficients[10] = {-1.7600413686 * pow(10, -2), 3.8921204975 * pow(10, -2), 1.8558770032 * pow(10, -5), -9.9457592874 * pow(10, -8), 3.1840945719 * pow(10, -10), -5.6072844889 * pow(10, -13), 5.6075059059 * pow(10, -16), -3.2020720003 * pow(10, -19), 9.7151147152 * pow(10, -23), -1.2104721275 * pow(10, -26)};
const double jVoltageCoefficients[9] = {0, 5.0381187815 * pow(10, -2), 3.0475836930 * pow(10, -5), -8.5681065720 * pow(10, -8), 1.3228195295 * pow(10, -10), -1.7052958337 * pow(10, -13), 2.0948090697 * pow(10, -16), -1.2538395336 * pow(10, -19), 1.5631725697 * pow(10, -23)};
const double kThermoCoefficients[3][10] = {{0,2.5173462 * pow(10, 1),-1.1662878,-1.0833638,-8.9773540 * pow(10, -1),-3.7342377 * pow(10, -1),-8.6632643 * pow(10, -2),-1.0450598 * pow(10, -2),-5.1920577 * pow(10, -4),0},
					   {0,2.508355 * pow(10, 1),7.860106 * pow(10, -2),-2.503131 * pow(10, -1),8.315270 * pow(10, -2),-1.228034 * pow(10, -2),9.804036 * pow(10, -4),-4.413030 * pow(10, -5),1.0577340 * pow(10, -6),-1.052755 * pow(10, -8)},
					   {-1.318058 * pow(10, 2),4.830222 * pow(10, 1),-1.646031,5.464731 * pow(10, -2),-9.650715 * pow(10, -4),8.802193 * pow(10, -6),-3.110810 * pow(10, -8),0,0,0}};
const double jThermoCoefficients[3][9] = {{0,1.9528268 * pow(10, 1),-1.2286185,-1.0752178,-5.9086933 * pow(10, -1),-1.7256713 * pow(10, -1),-2.8131513 * pow(10, -2),-2.3963370 * pow(10, -3),-8.3823321 * pow(10, -5)},
					  {0,1.978425 * pow(10, 1),-2.001204 * pow(10, -1),1.036969 * pow(10, -2),-2.549687 * pow(10, -4),3.585153 * pow(10, -6),-5.344285 * pow(10, -8),5.099890 * pow(10, -10),0},
					  {-3.1135818702 * pow(10, 3),3.00543684 * pow(10, 2),-9.94773230,1.70276630 * pow(10, -1),-1.43033468 * pow(10, -3),4.73886084 * pow(10, -6),0,0,0}};

static bool compareWith(int, int, ...);

int safeExtract(char* buf){
	if(buf)
		return buf[0];
	return INVAL_CMD;
}

static bool useACK(char id){
	return compareWith(id, 3, DAQC2, THERMO, TINKER);
}

static bool isValid(char id, char addr){
	return (addr >= 0 && addr <= 7) && compareWith(id, 6, DAQC2, THERMO, TINKER, DAQC, RELAY, MOTOR);
}

static bool compareWith(int id, int num, ...){
	va_list values;
	int i;

	va_start(values, num);

	for(i = 0; i < num; i++){
		int v = va_arg(values, int);
		if(id == v){
			va_end(values);
			return 1;
		}
	}
	va_end(values);
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

struct piplate pi_plate_init(char id, char addr){
	struct piplate plate = { };
	if(isValid(id, addr)){
		plate.id = id;
		plate.addr = addr;
		plate.mapped_addr = id + addr;
		plate.ack = useACK(id);
		plate.isValid = 1;
		if(getADDR(&plate) == plate.addr)
			return plate;
	}
	plate.isValid = 0;
	return plate;
}

bool getINT(){
	FILE *fp = fopen("/dev/PiPlates", "r");

	if(!fp)
		return NULL;

	int resp = ioctl(fileno(fp), PIPLATE_GETINT);

	fclose(fp);

	return resp;
}

/* Start of system commands: */

int getADDR(struct piplate* plate){
	if(plate->isValid){
		int resp = safeExtract(sendCMD(plate, 0x00, 0, 0, 1));
		return (resp > 0 ? resp - plate->id : resp);
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
	if(plate->isValid && compareWith(plate->id, 4, DAQC, DAQC2, MOTOR, THERMO))
		sendCMD(plate, 0x04, 0, 0, 0);
}

void intDisable(struct piplate* plate){
	if(plate->isValid && compareWith(plate->id, 4, DAQC, DAQC2, MOTOR, THERMO))
		sendCMD(plate, 0x05, 0, 0, 0);
}

int getINTflags(struct piplate* plate){
	if(plate->isValid && compareWith(plate->id, 3, DAQC, DAQC2, THERMO))
		return safeExtract(sendCMD(plate, 0x06, 0, 0, 1));
	return INVAL_CMD;
}

int getINTflag0(struct piplate* plate){
	if(plate->isValid && compareWith(plate->id, 1, MOTOR))
		return safeExtract(sendCMD(plate, 0x06, 0, 0, 1));
	return INVAL_CMD;
}

int getINTflag1(struct piplate* plate){
	if(plate->isValid && compareWith(plate->id, 1, MOTOR))
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
		if(compareWith(plate->id, 1, RELAY) && relay >= 1 && relay <=7){
			sendCMD(plate, 0x10, relay, 0, 0);
		}else if(compareWith(plate->id, 1, TINKER) && relay >= 1 && relay <= 2){
			sendCMD(plate, 0x10, relay - 1, 0, 0);
		}
	}
}

void relayOFF(struct piplate* plate, char relay){
	if(plate->isValid){
		if(compareWith(plate->id, 1, RELAY) && relay >= 1 && relay <= 7){
			sendCMD(plate, 0x11, relay, 0, 0);
		}else if(compareWith(plate->id, 1, TINKER) && relay >=1 && relay <= 2){
			sendCMD(plate, 0x11, relay - 1, 0, 0);
		}
	}
}

void relayTOGGLE(struct piplate* plate, char relay){
	if(plate->isValid){
		if(compareWith(plate->id, 1, RELAY) && relay >= 1 && relay <= 7){
			sendCMD(plate, 0x12, relay, 0, 0);
		}else if(compareWith(plate->id, 1, TINKER) && relay >= 1 && relay <= 2){
			sendCMD(plate, 0x12, relay - 1, 0, 0);
		}
	}
}

void relayALL(struct piplate* plate, char relays){
	if(plate->isValid){
		if(compareWith(plate->id, 1, RELAY)){
			if(relays >= 0 && relays <= 127)
				sendCMD(plate, 0x13, relays, 0, 0);
		}else if(compareWith(plate->id, 1, TINKER)){
			if(relays >= 0 && relays <= 3)
				sendCMD(plate, 0x13, relays, 0, 0);
		}
	}
}

int relaySTATE(struct piplate* plate, char relay){
	if(plate->isValid){
		if(compareWith(plate->id, 1, RELAY)){
			if(relay >= 1 && relay <= 7)
				return safeExtract(sendCMD(plate, 0x14, 0, 0, 1));
		}else if(compareWith(plate->id, 1, TINKER)){
			if(relay >= 1 && relay <= 2)
				return safeExtract(sendCMD(plate, 0x14, relay, 0, 0));
		}
	}
	return INVAL_CMD;
}

/* End of relay commands */

void setMODE(struct piplate* plate, char bit, char* mode){
	if(plate->isValid){
		if(compareWith(plate->id, 1, TINKER)){
			int numModes = 9;
			int modeSelect = numModes;
			bool channelGood = 0;
			int i;

			if (!strcmp(mode, "range")){
				if((bit==12) || (bit==34) || (bit==56) || (bit==78))
					channelGood = 1;
				else
					printf("Invalid channel pair. For range, must be 12, 34, 56, or 78.\n");
				bit = (bit>>1) / 10;//Maps the inputs to 0, 1, 2, and 3.
			}else{
				if(bit >= 1 && bit <= 8)
					channelGood = 1;
				else
					printf("Invalid channel. Must be 1 through 8.");
				bit--;
			}

			if(channelGood){
				for(i = 0; i < numModes; i++){
					if(!strcmp(mode, modes[i]))
						modeSelect = i;
				}
				if(!strcmp(mode, "led"))
					modeSelect = 3;

				if(modeSelect != numModes){
					if(pcaRequired[modeSelect] && bit >= 6){
						printf("This channel cannot support this mode.\n");
					}else{
						sendCMD(plate, 0x90, bit, modeSelect, 0);
					}
				}else{
					printf("Invalid mode.\n");
				}
			}
		}
	}
}


/* Start of digital output commands */

void setDOUTbit(struct piplate* plate, char bit){
	if(plate->isValid){
		if(compareWith(plate->id, 1, DAQC)){
			if(bit >= 0 && bit <= 6)
				sendCMD(plate, 0x10, bit, 0, 0);
		}else if(compareWith(plate->id, 1, DAQC2)){
			if(bit >= 0 && bit <= 7)
				sendCMD(plate, 0x10, bit, 0, 0);
		}else if(compareWith(plate->id, 1, TINKER)){
			if(bit >= 1 && bit <= 8)
				sendCMD(plate, 0x26, bit - 1, 0, 0);
		}
	}
}

void clrDOUTbit(struct piplate* plate, char bit){
	if(plate->isValid){
		if(compareWith(plate->id, 1, DAQC)){
			if(bit >= 0 && bit <= 6)
				sendCMD(plate, 0x11, bit, 0, 0);
		}else if(compareWith(plate->id, 1, DAQC2)){
			if(bit >= 0 && bit <= 7)
				sendCMD(plate, 0x11, bit, 0, 0);
		}else if(compareWith(plate->id, 1, TINKER)){
			if(bit >= 1 && bit <= 8)
				sendCMD(plate, 0x27, bit - 1, 0, 0);
		}
	}
}

void toggleDOUTbit(struct piplate* plate, char bit){
	if(plate->isValid){
		if(compareWith(plate->id, 1, DAQC)){
			if(bit >= 0 && bit <= 6)
				sendCMD(plate, 0x12, bit, 0, 0);
		}else if(compareWith(plate->id, 1, DAQC2)){
			if(bit >= 0 && bit <= 7)
				sendCMD(plate, 0x12, bit, 0, 0);
		}else if(compareWith(plate->id, 1, TINKER)){
			if(bit >= 1 && bit <= 8)
				sendCMD(plate, 0x28, bit - 1, 0, 0);
		}
	}
}

/* End of digital output commands */

/* Start of digital input commands */

int getDINbit(struct piplate* plate, char bit){
	if(plate->isValid){
		if(compareWith(plate->id, 1, TINKER))
			bit--;

		if(compareWith(plate->id, 3, DAQC, DAQC2, TINKER)){
			if(bit >= 0 && bit <= 7){
				int resp = safeExtract(sendCMD(plate, 0x20, bit, 0, 1));
				return (resp > 0 ? 1 : (resp < 0 ? INVAL_CMD : 0));
			}
		}
	}
	return INVAL_CMD;
}

int getDINall(struct piplate* plate){
	if(plate->isValid){
		if(compareWith(plate->id, 3, DAQC, DAQC2, TINKER)){
			return safeExtract(sendCMD(plate, 0x25, 0, 0, 1));
		}
	}
	return INVAL_CMD;
}

void enableDINint(struct piplate* plate, char bit, char edge){
	if(plate->isValid){
		if(compareWith(plate->id, 2, DAQC, DAQC2)){
			if(bit >= 0 && bit <= 7){
				if( edge == 'f' || edge == 'F' )
					sendCMD(plate, 0x21, bit, 0, 0);
				else if( edge == 'r' || edge == 'R' )
					sendCMD(plate, 0x22, bit, 0, 0);
				else if( edge == 'b' || edge == 'B' )
					sendCMD(plate, 0x23, bit, 0, 0);
			}
		}
	}
}

void disableDINint(struct piplate* plate, char bit){
	if(plate->isValid){
		if(compareWith(plate->id, 2, DAQC, DAQC2)){
			if(bit >= 0 && bit <= 7)
				sendCMD(plate, 0x24, bit, 0, 0);
		}
	}
}

/* End of digital input commands */


/* Start of board led commands */

//These ones are weird, I'll come back to them.

/* End of board led commands */


/* Calibration Constants / Flash Memory Functions */

int CalGetByte(struct piplate* plate, char ptr){
	if(plate->isValid){
		if(compareWith(plate->id, 2, DAQC2, THERMO)){
			return safeExtract(sendCMD(plate, 0xFD, 2, ptr, 1));
		}
	}
	return INVAL_CMD;
}

void CalPutByte(struct piplate* plate, char data){
	if(plate->isValid){
		if(compareWith(plate->id, 2, DAQC2, THERMO)){
			sendCMD(plate, 0xFD, 1, data, 0);
		}
	}
}

void CalEraseBlock(struct piplate* plate){
	if(plate->isValid){
		if(compareWith(plate->id, 2, "DAQC2", THERMO)){
			sendCMD(plate, 0xFD, 0, 0, 0);
		}
	}
}

/* Calibration Constants / Flash Memory Functions */

/* Start of DAQC2 Oscilloscope Commands */

void startOSC(struct piplate* plate){
	if(plate->isValid){
		if(compareWith(plate->id, 1, DAQC2)){
			plate->osc = (struct oscilloscope*)calloc(1, sizeof(struct oscilloscope));

			plate->osc->c1State = 1;
			plate->osc->sRate = 9;

			sendCMD(plate, 0xA1, 0, 0, 0);
		}
	}
}

void stopOSC(struct piplate* plate){
	if(plate->isValid){
		if(compareWith(plate->id, 1, DAQC2)){
			if(!plate->osc)
				free(plate->osc);

			sendCMD(plate, 0xA0, 0, 0, 0);
		}
	}
}

void setOSCchannel(struct piplate* plate, bool c1, bool c2){
	if(plate->isValid){
		if(compareWith(plate->id, 1, DAQC2)){
			plate->osc->c1State = c1;
			plate->osc->c2State = c2;
			sendCMD(plate, 0xA2, c1, c2, 0);
		}
	}
}

/*
* Rate:		Samples/sec:
* 0:		100
* 1:		200
* 2:		500
* 3:		1,000
* 4:		2,000
* 5:		5,000
* 6;		10,000
* 7:		20,000
* 8:		50,000
* 9:		100,000
* 10:		200,000
* 11:		500,000
* 12:		1,000,000
* Note: #12 can only be used with a single channel input.
*/

void setOSCsweep(struct piplate* plate, char rate){
	if(plate->isValid){
		if(compareWith(plate->id, 1, DAQC2)){
			if(rate >= 0 && rate <= 12)
				sendCMD(plate, 0xA3, rate, 0, 0);
		}
	}
}

void getOSCtraces(struct piplate* plate){
	if(plate->isValid){
		if(compareWith(plate->id, 1, DAQC2)){
			int i;

			char cCount = plate->osc->c1State + plate->osc->c2State;
			char* resp = sendCMD(plate, 0xA4, 0, 0, cCount*2048);

			if(resp){
				if(cCount == 2){
					for(i = 0; i < 1024; i++){
						plate->osc->trace1[i] = resp[4*i]*256 + resp[4*i+1];
						plate->osc->trace2[i] = resp[4*i+2]*256 + resp[4*i+3];
					}
				}else{
					if(plate->osc->c1State){
						for(i = 0; i < 1024; i++)
							plate->osc->trace1[i]=resp[2*i]*256+resp[2*i+1];
					}else{
						for(i = 0; i < 1024; i++)
							plate->osc->trace2[i]=resp[2*i]*256+resp[2*i+1];
					}
				}
			}
		}
	}
}

void setOSCtrigger(struct piplate* plate, char channel, char* type, char* edge, int level){
	if(plate->isValid){
		if(compareWith(plate->id, 1, DAQC2)){
			char option = 0;
			if(channel >= 1 && channel <= 2)
				option = 128*(channel-1);

			if(!strcmp(type, "normal"))//Options: "normal", "auto"
				option += 64;

			if(!strcmp(edge, "falling"))//Options: "rising", "falling"
				option += 32;

			if(level >= 0 && level <= 4095)
				sendCMD(plate, 0xA6, option + (level>>8), level&0xFF, 0);
		}
	}
}

void trigOSCnow(struct piplate* plate){
	if(plate->isValid){
		if(compareWith(plate->id, 1, DAQC2)){
			sendCMD(plate, 0xA7, 0, 0, 0);
		}
	}
}

void runOSC(struct piplate* plate){
	if(plate->isValid){
		if(compareWith(plate->id, 1, DAQC2)){
			sendCMD(plate, 0xA5, 0, 0, 0);
		}
	}
}

/* End of DAQC2 Oscilloscope Commands */

/* Start of stepper motor functions */

void stepperINIT(struct piplate* plate){
	int i = 0;

	plate->stm = (struct stepperMotorParams*)calloc(2, sizeof(struct stepperMotorParams));
	for(i = 0; i < 2; i++){
		plate->stm[i].dir = CW;
		plate->stm[i].resolution = FULL_STEP;
		plate->stm[i].rate = 500;
		plate->stm[i].acc = 0;
	}
}

void stepperENABLE(struct piplate* plate){
	if(plate->isValid){
		if(compareWith(plate->id, 1, DAQC2))
			sendCMD(plate, 0xB1, 0, 0, 0);
	}
}

void stepperDISABLE(struct piplate* plate){
	if(plate->isValid){
		if(compareWith(plate->id, 1, DAQC2))
			sendCMD(plate, 0xB0, 0, 0, 0);
	}
}

void stepperINTenable(struct piplate* plate, char motor){
	if(plate->isValid){
		if(compareWith(plate->id, 1, DAQC2)){
			if(motor >= 1 && motor <= 2)
				sendCMD(plate, 0xB7, motor-1, 0, 0);
		}
	}
}

void stepperINTdisable(struct piplate* plate, char motor){
	if(plate->isValid){
		if(compareWith(plate->id, 1, DAQC2)){
			if(motor >= 1 && motor <= 2)
				sendCMD(plate, 0xB8, motor-1, 0, 0);
		}
	}
}

void stepperCONFIG(struct piplate* plate, char motor, char direction, char resolution, int rate, char acceleration){
	if(plate->isValid){
		if(!plate->stm)
			stepperINIT(plate);

		if(compareWith(plate->id, 1, MOTOR)){
			if(motor >= 1 && motor <= 2 && (direction == CW || direction == CCW) && resolution >= 0 && resolution <= 3 && rate >= 1 && rate <= 2000 && acceleration >= 0 && acceleration <= 10){
				int param1 = 0;
				int param2 = rate & 0x00FF;
				int cmd = 0x10 + motor - 1;
				int increment;

				plate->stm[motor].dir = direction;
				plate->stm[motor].resolution = resolution;
				plate->stm[motor].rate = rate;
				plate->stm[motor].acc = acceleration;

				if (direction == CW)
					param1 = 0x40;
				param1 += (resolution << 4);
				param1 += (rate >> 8);

				sendCMD(plate, cmd, param1, param2, 0);

				if(acceleration == 0)
					increment = 0;
				else
					increment = (int)(1024 * rate / (acceleration*2000) + 0.5);

				param1 = 0x80 + (increment>>8);
				param2 = increment & 0x00FF;
				sendCMD(plate, cmd, param1, param2, 0);
			}
		}
	}
}

void stepperDIR(struct piplate* plate, char motor, char direction){
	if(plate->isValid){
		if(!plate->stm)
			stepperINIT(plate);

		if(compareWith(plate->id, 1, DAQC2)){
			if(motor >= 1 && motor <= 2){
				sendCMD(plate, 0xB3, motor - 1, direction, 0);
			}
		}else if(compareWith(plate->id, 1, MOTOR)){
			stepperCONFIG(plate, motor, direction, plate->stm[motor].resolution, plate->stm[motor].rate, plate->stm[motor].acc);
		}
	}
}

void stepperRATE(struct piplate* plate, char motor, int rate, char resolution){
	if(plate->isValid){
		if(!plate->stm)
			stepperINIT(plate);

		if(compareWith(plate->id, 1, DAQC2)){
			if(motor >= 1 && motor <= 2 && rate >= 0 && rate <= 500 && resolution >= FULL_STEP && resolution <= HALF_STEP){
				int rateInc = (int)(rate*pow(2, 13)/1000.0 + 0.5);
				int param1 = ((motor-1)<<7)+(rateInc>>8);
				int param2;

				if(resolution == HALF_STEP)
					param1 |= 0x40;
				param2 = rateInc&0xFF;
				sendCMD(plate, 0xB2, param1, param2, 0);
			}
		}else if(compareWith(plate->id, 1, MOTOR)){
			stepperCONFIG(plate, motor, plate->stm[motor].dir, resolution, rate, plate->stm[motor].acc);
		}
	}
}

void stepperACC(struct piplate* plate, char motor, char acceleration){
	if(plate->isValid){
		if(!plate->stm)
			stepperINIT(plate);

		if(compareWith(plate->id, 1, MOTOR)){
			stepperCONFIG(plate, motor, plate->stm[motor].dir, plate->stm[motor].resolution, plate->stm[motor].rate, acceleration);
		}
	}
}

void stepperMOVE(struct piplate* plate, char motor, int steps){
	if(plate->isValid){
		if(compareWith(plate->id, 1, DAQC2)){
			if(motor >= 1 && motor <= 2 && steps >= -16383 && steps <= 16383){
				bool stepSign = (steps > 0 ? 1 : 0);
				int param1 = ((motor - 1) << 7) + (stepSign << 6) + (steps>>8);
				int param2 = abs(steps) & 0xFF;
				sendCMD(plate, 0xB4, param1, param2, 0);
			}
		}else if(compareWith(plate->id, 1, MOTOR)){
			if(motor >= 1 && motor <= 2 && steps <= 65535){
				char cmd = 0x12 + motor - 1;
				int param1 = steps>>8;
				int param2 = steps&0xFF;
				sendCMD(plate, cmd, param1, param2, 0);
			}
		}
	}
}

void stepperJOG(struct piplate* plate, char motor){
	if(plate->isValid){
		if(compareWith(plate->id, 1, DAQC2)){
			if(motor >= 1 && motor <= 2)
				sendCMD(plate, 0xB5, motor-1, 0, 0);
		}else if(compareWith(plate->id, 1, MOTOR)){
			if(motor >= 1 && motor <= 2)
				sendCMD(plate, 0x14 + motor - 1, 0, 0, 0);
		}
	}
}

void stepperSTOP(struct piplate* plate, char motor){
	if(plate->isValid){
		if(compareWith(plate->id, 1, DAQC2)){
			if(motor >= 1 && motor <= 2)
				sendCMD(plate, 0xB6, motor - 1, 0, 0);
		}else if(compareWith(plate->id, 1, MOTOR)){
			if(motor >= 1 && motor <= 2)
				sendCMD(plate, 0x16 + motor - 1, 0, 0, 0);
		}
	}
}

void stepperOFF(struct piplate* plate, char motor){
	if(plate->isValid){
		if(compareWith(plate->id, 1, DAQC2)){
			if(motor >= 1 && motor <= 2)
				sendCMD(plate, 0xBA, motor - 1, 0, 0);
		}else if(compareWith(plate->id, 1, MOTOR)){
				if(motor >= 1 && motor <= 2)
					sendCMD(plate, 0x1E + motor - 1, 0, 0, 0);
		}
	}
}

/* End of stepper motor functions */

/* Start of thermo functions */

//Convert 32 bit number to a double
double binaryToDouble(int* list){
	int i;
	int polarity = 1;
	int exp;
	double frac;
	int expsign = 1;
	int val = list[0];

	for(i = 0; i < 3; i ++){
		val = val << 8;
		val += list[i + i];
	}

	if(val & pow(2, 31) != 0)//Read first bit
		polarity = -1;

	exp = ((val>>24)&0x7F)-64;
	if(exp < 0)
		expsign = -1;

	val=val&((int)pow(2, 24)-1);

	frac = (double)val/((double)(pow(2, 24)-1))*expsign;
	return pow(10.0, exp + frac)*polarity;
}

void tempINIT(struct piplate* plate){
	int i;

	plate->tmp = (struct tempParams*)calloc(1, sizeof(struct tempParams));
	for(i = 0; i < 12; i ++){
		plate->tmp->scale[i] = KELVINS;//Set scales
	}

	for(i = 0; i < 8; i ++){
		plate->tmp->type[i] = 'k';//Set types
	}

	//Calibration data from flash memory

	int values[4];
	for(i = 0; i < 4; i ++){
		values[i] = CalGetByte(plate, i);
	}
	plate->tmp->calBias = binaryToDouble(values);

	for(i = 0; i < 8; i++){
		int j;
		for(j = 0; j < 4; j++){
			values[j]=CalGetByte(plate, 8*i+j+4);
		}
		plate->tmp->calOffset[i]=binaryToDouble(values);
		for(j = 4; j < 8; j++){
			values[j-4]=CalGetByte(plate, 8*i+j+4);
		}
		plate->tmp->calScale[i]=binaryToDouble(values);
	}
}

void setSCALE(struct piplate* plate, char channel, char scale){
	if(plate->isValid){
		if(compareWith(plate->id, 1, THERMO)){
			if(!plate->tmp)
				tempINIT(plate);

			if(channel >= 1 && channel <= 12){
				if(scale == KELVINS || scale == CELSIUS || scale == FAHRENHEIT)
					plate->tmp->scale[channel - 1] = scale;
			}
		}
	}
}

void setTYPE(struct piplate* plate, char channel, char type){
	if(plate->isValid){
		if(compareWith(plate->id, 1, THERMO)){
			if(!plate->tmp)
				tempINIT(plate);

			if(channel >= 1 && channel <= 8){
				if(type == 'k' || type == 'j')
					plate->tmp->type[channel - 1] = type;
			}
		}
	}
}

char getTYPE(struct piplate* plate, char channel){
	if(plate->isValid){
		if(compareWith(plate->id, 1, THERMO)){
			if(!plate->tmp)
				tempINIT(plate);

			if(channel >= 1 && channel <= 8)
				return plate->tmp->type[channel - 1];
		}
	}
}

char getSCALE(struct piplate* plate, char channel){
	if(plate->isValid){
		if(compareWith(plate->id, 1, THERMO)){
			if(!plate->tmp)
				tempINIT(plate);

			if(channel >= 1 && channel <= 12)
				return plate->tmp->scale[channel - 1];
		}
	}
	return INVAL_CMD;
}

double getTEMP(struct piplate* plate, char channel){
	if(plate->isValid){
		if(compareWith(plate->id, 1, THERMO)){
			if(!plate->tmp)
				tempINIT(plate);

			if(channel >= 1 && channel <= 12){
				int Tvals[2];
				char* resp = sendCMD(plate, 0x70, channel - 1, 0, 4);
				double temp;

				channel--;

				if(!resp)
					return INVAL_CMD;

				Tvals[0] = resp[0] * 256 + resp[1];//T channel data
				Tvals[1] = resp[2] * 256 + resp[3];//Cold junction data

				if(channel > 7){
					int t = Tvals[0];
					if(t > 0x8000){//It's negative, take the 2's complement.
						t = t^0xFFFF;
						t = -(t + 1);
					}
					temp = t / 16.0;//Celsius
				}else{
					int i;
					double a = -0.00347;
					double b = -10.888;
					double c = 1777.3;

					double coldJunctionV = 0;
					double vMeas;
					double vHot;
					int k = 1;

					temp = Tvals[1] * 2400.0/65535.0;//Convert cold junction data to voltage
					c -= temp;
					temp = (-b-sqrt(pow(b, 2) - (4*a*c)))/(2 * a) + 30.0; //Convert voltage to temperature

					printf("voltage to temp: %f\n", temp);

					if(plate->tmp->type[channel] == 'k'){
						for(i = 0; i < 10; i ++){
							coldJunctionV += kVoltageCoefficients[i]*pow(temp, i);
						}
					}else{
						for(i = 0; i < 9; i ++){
							coldJunctionV += jVoltageCoefficients[i]*pow(temp, i);
						}
					}

					printf("to voltage: %f\n", coldJunctionV);

					vMeas=((Tvals[0]*2.4/65535.0)-plate->tmp->calOffset[channel])/plate->tmp->calScale[channel]*1000;
					vHot=vMeas+coldJunctionV-(plate->tmp->calBias * 1000.0);

					printf("to hot: %f\n", vHot);

					k = (vHot < 0) ? 0 : ((vHot > 20.644) ? 2 : 1);

					temp = 0;
					if(plate->tmp->type[channel] == 'k'){
						for(i = 0; i < 10; i++){
							temp += kThermoCoefficients[k][i]*pow(vHot, i);
						}
					}else{
						for(i = 0; i < 10; i ++){
							temp += jThermoCoefficients[k][i]*pow(vHot, i);
						}
					}

					printf("and finally to C: %f\n", temp);
				}
				if(plate->tmp->scale[channel] == KELVINS)
					temp += 273.15;
				else if(plate->tmp->scale[channel] == FAHRENHEIT)
					temp = temp * 1.8 + 32.0;

				temp = ((int) (temp * 1000)) / 1000.0;//Round
				return temp;
			}
		}
	}
	return INVAL_CMD;
}

double getCOLD(struct piplate* plate){
	if(plate->isValid){
		if(compareWith(plate->id, 1, THERMO)){
			if(!plate->tmp)
				tempINIT(plate);

		}
	}
}

double getRAW(struct piplate* plate, char channel){
	if(plate->isValid){
		if(compareWith(plate->id, 1, THERMO)){
			if(!plate->tmp)
				tempINIT(plate);

		}
	}
}

/* End of thermo functions */

