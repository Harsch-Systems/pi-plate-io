#include <stdio.h>
#include <unistd.h>
#include "plateio.h"

void main(){
	struct piplate plate = pi_plate_init(TINKER, 5);

	setMODE(&plate, 8, "servo");
	sleep(0.5);

	while(1){
		setSERVO(&plate, 8, 0.0);
		sleep(2);
		setSERVO(&plate, 8, 180.0);
		sleep(2);
	}
}
