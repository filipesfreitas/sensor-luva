#include <math.h>
#include <position_process.h>
#define gyro_factor 16.4
#define acc_factor 2048

void orientation_estimation(int16_t buffer[],float packet[10]) {

	float converted_buffer[12];

	/* PHI	*/
	packet[0] = acos(buffer[4]/(sqrt(pow(buffer[2],2)+pow(buffer[3],2)+pow(buffer[4],2))));
	packet[2] = acos(buffer[10]/(sqrt(pow(buffer[8],2)+pow(buffer[9],2)+pow(buffer[10],2))));
	
	/* THETA */
	if (buffer[1]==0) packet[1] = 0;
	else packet[1] = atan(buffer[2]/buffer[3]);

	if (buffer[9]==0)	packet[3] = 0;
	else packet[3] = atan(buffer[8]/buffer[9]);
	
	packet[4] = buffer[5]  ;
	packet[5] = buffer[6]  ;
	packet[6] = buffer[7]  ;
	packet[7] = buffer[11] ;
	packet[8] = buffer[12] ;
	packet[9] = buffer[13] ;

	}