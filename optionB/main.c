#include <stdio.h>
#include "plateio.h"

void main(){
	struct piplate plate = { };
	int i;

	pi_plate_init(&plate, "DAQC", 1);

	printf("id: %x\n", getDINbit(&plate, 0));
}
