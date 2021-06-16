#ifndef POSITION_PROCESS_H
#define POSITION_PROCESS_H

#include <stdint.h>
#include "esp_timer.h"
#include <string.h>
#include <math.h>
#include <misc.h>
#include <i2c_handler.h>
#include "esp_log.h"

static const char *TAG = "TAG_ESP";

#define gyro_factor 16.4
#define acc_factor 2048
#define degre_conv 180/M_PI
#define time_constant 10E-3
#define period 50E-3
#define R 4700 	/* Resistance of voltage divider on ohms */
#define Vinput 3300 /* Input voltage in mV */

typedef struct 
{
		int16_t accelx;
		int16_t accely;
		int16_t accelz;

		int16_t gyrox;
		int16_t gyroy;
		int16_t gyroz;
	
		int master_num;
		int finger;

} raw_data;

typedef struct {
	float phi;
	float theta;
}orientation;

typedef struct {
	orientation proximal;
	orientation medial;
	orientation distal;
	orientation metacarpophalangeal;
	int16_t pressure;
	
}finger;

typedef struct {
	finger fingers[5];
	orientation frame_reference;
}Glove;

void orientation_estimation(raw_data metacarpo,raw_data proximal,Glove* glove,int i);
void reference_frame_orientation(raw_data reference,Glove* glove);
void calibration(Glove* glove);
void initialization(Glove* glove);
void buffer_arrange(Glove* glove, char message[]);
void raw_data_zero(raw_data* member);
void buffer_raw_data(raw_data* test1,raw_data* test2);
#endif