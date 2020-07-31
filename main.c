#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "plateio.h"

void main(){
	struct piplate plate = pi_plate_init(THERMO, 3);

	int i = 1;
	while(i){
		char* id = getID(&plate);
		printf("id: %s\n", id);
		if(strcmp(id, "Pi-Plate THERMOplate"))
			i = 0;
	}
/*
	struct piplate plate = pi_plate_init(TINKER, 5);

	setMODE(&plate, 8, "servo");
	sleep(0.5);

	while(1){
		setSERVO(&plate, 8, 0.0);
		sleep(2);
		setSERVO(&plate, 8, 180.0);
		sleep(2);
	}
*/
}
