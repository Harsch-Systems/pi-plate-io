#include <stdio.h>
#include "plateio.h"

void main(){
	struct piplate plate = { };
	pi_plate_init(&plate, "DAQC", 1);

	setDOUTbit(&plate, 1);

	//printf("id: %s\n", getID(&plate));
}
