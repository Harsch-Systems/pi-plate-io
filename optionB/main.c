#include <stdio.h>
#include "plateio.h"

void main(){
	struct piplate plate = { };
	pi_plate_init(&plate, "RELAY", 0);

	relayTOGGLE(&plate, 4);

	printf("value: %d\n", relaySTATE(&plate, 4));
}
