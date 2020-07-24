#include <stdio.h>
#include <unistd.h>
#include "plateio.h"

void main(){
	struct piplate plate = pi_plate_init(DAQC, 1);

	enableSWint(&plate);
	intEnable(&plate);

	while(1){
		bool interrupt = getINT();
		if(!interrupt){
			printf("int: %f\n", interrupt);
			printf("flags: %x\n", getINTflags(&plate));
		}
	}
}
