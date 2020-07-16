#include <stdio.h>
#include "plateio.h"

void main(){
	struct piplate plate = pi_plate_init(MOTOR, 3);
	int i;

	printf("id: %s\n", getID(&plate));
}
