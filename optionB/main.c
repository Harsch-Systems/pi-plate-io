#include <stdio.h>
#include "plateio.h"

void main(){
	struct piplate plate = { };
	pi_plate_init(&plate, "DAQC", 2);

	printf("value: %d\n", getADDR(&plate));
}
