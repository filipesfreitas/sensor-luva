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
/**
 * Gyroscpe convertion
 */
#define gyro_factor 16.4
#define acc_factor 2048
#define degre_conv 180/M_PI
#define radconv M_PI/180
#define time_constant .1
#define period 70E-3
#define R 4700 	/* Resistance of voltage divider on ohms */
#define Vinput 3300 /* Input voltage in mV */
#define channels 1
#define gyrosens radconv/16.4
typedef struct 
{
		int accelx;
		int accely;
		int accelz;

		int gyrox;
		int gyroy;
		int gyroz;
	
		int master_num;
		int finger;
} raw_data;

typedef struct {
	float phi;
	float theta;
}orientation;

typedef struct {
	orientation proximal;
	orientation distal;
	orientation metacarpo;
	float pressure;
}finger;

typedef struct {
	finger fingers[5];
	orientation frame_reference;
}Glove;

/**
 * @brief      fix orientation angle consedering the limitations of using inverse trigonometrics fuction.
 *
 * @param[in]  raw    The raw data of the IMU, it is used to determine the quadrant in the unitary circle that the IMU was.
 * @param[in]  angle  The estimative of the angle produced by the system.
 *
 * @return     the correction of the angle rotated 90 degres.
 */
float fix_orientation(raw_data raw,float angle);
/**
 * @brief      Update the orientation of the position of a given finger.
 *
 * @param[in]  metacarpo  matacarpal falange raw data, i.e acceleration and gyroscope data.
 * @param[in]  proximal   proximal falange raw data, i.e acceleration and gyroscope data.
 * @param      glove      Structure representing the hand posture captured by the glove device.
 * @param[in]  i          finger indication for update orientation.
 */
void orientation_estimation(raw_data metacarpo,raw_data proximal,Glove* glove,int i);
/**
 * @brief      Update of the orientation of the reference frame.
 *
 * @param[in]  reference  Raw data contaning acceleration and gyroscope input.
 * @param      glove      Structure representing the hand posture captured by the glove device.
 */
void reference_frame_orientation(raw_data reference,Glove* glove);
/**
 * @brief      Calibration funtion, take mesure of aceleration of every device connected and estimate a angle for future reference.
 *
 * @param      glove  Structure representing the hand posture captured by the glove device.
 */
void calibration(Glove* glove);
/**
 * @brief      Call i2c_imu_setup for initialize every device connected, i.e IMU, for turning them up and do configure the parameters of internal filters bandwidth, powerup type and etc.
 *
 * @param      glove  Structure representing the hand posture captured by the glove device.
 */
void initialization(Glove* glove);
/**
 * @brief      Buffer char[] type for configure the shape of the packge to be displayed or send to device connected throug WiFi.
 *
 * @param      glove    Structure representing the hand posture captured by the glove device.
 * @param      message  Char* type parameter buffer for storing hand position.
 */
void buffer_arrange(Glove* glove, char message[]);
/**
 * @brief      Zero input for every member of member.	
 *
 * @param      member  Structured to be initialized, values.
 */
void raw_data_zero(raw_data* member);
/**
 * @brief      Debugger fucntion that put on stdout, on monitor when the device is connected to a computer,
 * 						 for visualization of raw data.	
 * @param      test1 	Raw data of first device attached.	 
 * @param      test2  Raw data of first device attached.
 */
void buffer_raw_data(raw_data* test1,raw_data* test2);
/**
 * @brief      Take glove structure and realize fram trasnformation for the reference frame!
 *  
 * @param      glove  The glove structure.
 * @param[in]  i      finger index.
 */
/**
 * @brief      setup sensors
 */
void setup_sensors();

#endif