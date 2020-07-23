#include <stdio.h>
#include <unistd.h>
#include "plateio.h"

void main(){
	struct piplate plate = pi_plate_init(DAQC2, 4);

	double val = getADC(&plate, 8);

	printf("val8: %f\n", val);

/*
	startOSC(&plate);//Start the plate
	setOSCchannel(&plate, 1, 0);//Set the channel

	setOSCtrigger(&plate, 1, "normal", "rising", 2048);//Set the trigger

	setOSCsweep(&plate, 8);//Set the sample rate
	intEnable(&plate);//Enable interrupts (oscilloscope mode uses interrupts by default)
	runOSC(&plate);

	bool ready = 0;
	while(!ready){
		if(!getINT()){
			ready = 1;
			getINTflags(&plate);//Clear interrupt flags
		}
	}

	getOSCtraces(&plate);//Get the recorded values

	printf("num:\tvalue:");
	for(i = 0; i < 1000; i++){
		printf("%d\t%d\n", i, plate.osc->trace1[i]);//Print out the first 1000 values
	}

	stopOSC(&plate);
*/
}
