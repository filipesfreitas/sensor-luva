#include <math.h>
#include "position_process.h"

float * orientation_estimation(int16_t accel_buffer[]) {
	/* Orientation based on estimation*/
	float packet[4];

	// aceleration
	/* PHI	*/
	packet[0] = acos(accel_buffer[2]/(sqrt(pow(accel_buffer[0],2),pow(accel_buffer[1],2),pow(accel_buffer[2],2))));
	packet[2] = acos(accel_buffer[5]/(sqrt(pow(accel_buffer[3],2),pow(accel_buffer[4],2),pow(accel_buffer[5],2))));
	
	/* THETA */
	if (accel_buffer[1]==0) packet[1] = 0;
	else packet[1] = atan(accel_buffer[0]/accel_buffer[1]);

	if (accel_buffer[4]==0)	packet[3] = 0;
	else packet[3] = atan(accel_buffer[3]/accel_buffer[4]);
	
	return packet;
}


