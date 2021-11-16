/** @file */ 
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "protocol_examples_common.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include <string.h>
#include <sys/param.h>
#include <math.h>
#include <unistd.h>
#include <esp_system.h>
#include <time.h>
#include <sys/time.h>
#include "esp_timer.h"
#include <misc.h>
#include <position_process.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <i2c_handler.h>
#include <softap.h>

#define PORT CONFIG_EXAMPLE_PORT
/* Event group bit set*/
#define PORT0ADX  		( 1UL << 0UL )  /* Event bit 0, read two sensors at the port0 i2c */
#define PORT1ADX  		( 1UL << 1UL ) 	/* Event bit 1, read two sensors at the port1 i2c */
#define UDP  	  			( 1UL << 2UL )  /* Event bit 2, UDP server */
#define DISPBUFFER 		PORT0ADX|PORT1ADX  /* Display buffer for channel reading*/
#define SYNCHRONIZED 	( 1UL << 4UL )  /* Flag Sync, change the channel of the mux*/
#define STOPAQ 				( 1UL << 5UL )
#define STARTAQ				( 1UL << 6UL )
#define PRINTAQ				(1UL << 7UL)
#define RESTARTAQ			UDP

static TaskHandle_t xTaskUDP = NULL,
xTaskREF = NULL,
xTaskI2C0 = NULL,
xTaskSYNCH = NULL,
xTaskDISP=NULL;

/* Event group instantiation*/
EventGroupHandle_t xEventGroup;

/*Glove instance*/
Glove* glove;

const TickType_t xDelay = 500/ portTICK_PERIOD_MS;/*delay defined by period ms*/
const TickType_t sample_time = 50/ portTICK_PERIOD_MS;/*delay defined by period ms*/


/**
 * @brief      Display the buffer of the package when the device is in debugger mode.
 *
 * @param      pvParameters  Parameter for the task.
 */
static void disp_buf(void * pvParameters)
{	
	char tx_buffer_msg[1024]={'0'};
	while(1){
		buffer_arrange(glove,tx_buffer_msg);
		//printf("%s\n",tx_buffer_msg);
		xEventGroupSync(xEventGroup,RESTARTAQ,STOPAQ,xDelay);
	}
}

/**
 * @brief      Task to read the values given by the IMUs and call the orientation estimate function to update the values of the hand posture stored in glove structure.
 *
 * @param      pvParameters  Number given to the task for i2c master number.
 */
static void i2c_task0(void *pvParameters)
{
	/* IMU CONFIG*/
	int finger=0;
	int ret, ret1;
	int count =1;
	raw_data metacarpo, proximal;
	uint8_t sensor[14];
	uint8_t sensor2[14];

	float buffer;
	i2c_port_t master_num=(int) pvParameters;

	memset(sensor,0,14);
	memset(sensor2,0,14);
	raw_data_zero(&metacarpo);
	raw_data_zero(&proximal);
	xEventGroupWaitBits(xEventGroup,STARTAQ,pdFALSE,pdTRUE,portMAX_DELAY);

	while (1) {	
		xEventGroupWaitBits(xEventGroup,PORT1ADX,pdTRUE,pdTRUE,xDelay);
		ret  = i2c_master_read_slave(master_num, SLAVE1_ADD,START_READ_ADD,sensor, 14);
		ret1 = i2c_master_read_slave(master_num, SLAVE2_ADD,START_READ_ADD,sensor2, 14);

		if (ret != ESP_OK || ret1 != ESP_OK) {
			ESP_LOGW(TAG,"\vProblem at master nº: %d\tSensor %s timed out...skip...\v",master_num,
				ret  != ESP_OK ? "SENSOR 0x68" :
				ret1 != ESP_OK ? "SENSOR 0x69" : "None");
		} 
		else if (ret == ESP_OK && ret1 == ESP_OK) {
			//ESP_LOGW(TAG, "I2C0");	
			metacarpo.accelx	= (int16_t)((sensor[0]   << 8) | sensor[1])-83;	/* ACCEL X */
			metacarpo.accely	= (int16_t)((sensor[2]   << 8) | sensor[3])-33;	/* ACCEL y */
			metacarpo.accelz	= (int16_t)((sensor[4]   << 8) | sensor[5])-15;	/* ACCEL z */
			metacarpo.gyrox		= (int16_t)((sensor[8]   << 8) | sensor[9])/gyro_factor;	/* GIRO X  */
			metacarpo.gyroy		= (int16_t)((sensor[10]  << 8) | sensor[11])/gyro_factor; /* GIRO Y  */
			metacarpo.gyroz		= (int16_t)((sensor[12]  << 8) | sensor[13])/gyro_factor; /* GIRO Z  */
			proximal.accelx		= (int16_t)((sensor2[0]  << 8) | sensor2[1])+180; /* ACCEL X */
			proximal.accely		= (int16_t)((sensor2[2]  << 8) | sensor2[3])+30;	/* ACCEL Y */
			proximal.accelz		= (int16_t)((sensor2[4]  << 8) | sensor2[5])+100;	/* ACCEL Z */
			proximal.gyrox		= (int16_t)((sensor2[8]  << 8) | sensor2[9])/gyro_factor;	/* GIRO X  */
			proximal.gyroy		= (int16_t)((sensor2[10] << 8) | sensor2[11])/gyro_factor;/* GIRO Y  */
			proximal.gyroz		= (int16_t)((sensor2[12] << 8) | sensor2[13])/gyro_factor;/* GIRO Z  */

			/* Pressão através do potenciômetro*/
			buffer = adc_read(finger,adc_chars);
			glove -> fingers[finger].pressure = buffer;
			orientation_estimation(metacarpo,proximal,glove,finger);
		}
		xEventGroupSetBits(xEventGroup,SYNCHRONIZED);

	}
	vTaskDelete(NULL);
}

/**
 * @brief      Take raw samples from the reference frame device and call for orientation estimatr function
 *
 * @param      pvParameters  Task parameters.
 */
static void i2c_task_reference_frame(void *pvParameters)
{

	int ret;
	raw_data ref,r;
	uint8_t sensor[14];

	i2c_port_t master_num = (int) pvParameters;
	memset(sensor,0,14);
	raw_data_zero(&ref);
	raw_data_zero(&r);
	xEventGroupWaitBits(xEventGroup,STARTAQ,pdFALSE,pdTRUE,portMAX_DELAY);
	while (1) {	
		xEventGroupWaitBits(xEventGroup,PORT0ADX,pdTRUE,pdTRUE,xDelay);
		ret  = i2c_master_read_slave(master_num, SLAVE1_ADD,START_READ_ADD,sensor, 14);
		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "I2C ERROR");									
		} 
		else {
			//ESP_LOGW(TAG, "I2C1");	
			ref.master_num  = master_num;
			ref.finger  = 0;
			ref.accelx  = 	(int16_t)((sensor[2]   << 8)  | sensor[3])	+114;
			ref.accely  = 	(int16_t)((sensor[0]   << 8)  | sensor[1])	-4; 
			ref.accelz  = 	(int16_t)((sensor[4]   << 8)  | sensor[5])	+121; 
			ref.gyrox   = 	(int16_t)((sensor[10]  << 8)  | sensor[11]	 )	/gyro_factor;
			ref.gyroy   = (	(int16_t)((sensor[8]   << 8)  | sensor[9])+1 )	/gyro_factor; 
			ref.gyroz   = (	(int16_t)((sensor[12]  << 8)  | sensor[13])+1)	/gyro_factor;
			reference_frame_orientation(ref,glove);
		}
		xEventGroupSetBits(xEventGroup,SYNCHRONIZED);

	}
}

/**
 * @brief      UDP task, handlle the connection and send gloves information to a client connected
 *
 * @param      pvParameters  The pv parameters
 */
static void udp_server_task(void *pvParameters)
{
	char rx_buffer[128];
	char tx_buffer[256];
	char addr_str[128];
	int addr_family;
	int ip_protocol;
	memset(tx_buffer,'0',256*sizeof(char));
	while(1){
		struct sockaddr_in dest_addr;
		dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
		dest_addr.sin_family = AF_INET;
		dest_addr.sin_port = htons(PORT);
		addr_family = AF_INET;
		ip_protocol = IPPROTO_IP;
		inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
		int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
		if (sock < 0) {
			ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
			break;
		}
		ESP_LOGI(TAG, "Socket created");

		int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
		if (err < 0) {
			ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
		}
		ESP_LOGI(TAG, "Socket bound, port %d", PORT);
		while(1){
			ESP_LOGI(TAG, "Waiting for data");
			struct sockaddr_storage source_addr; 
			socklen_t socklen = sizeof(source_addr);
			int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

			if (len < 0) {
				ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
				break;
			}
			else {
				if (source_addr.ss_family == PF_INET) {
					inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
				} else if (source_addr.ss_family == PF_INET6) {
					inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
				}
				rx_buffer[len] = 0; 
				ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
				ESP_LOGI(TAG, "%s", rx_buffer);
				while(1){
					xEventGroupSync(xEventGroup,RESTARTAQ,STOPAQ,xDelay);
					buffer_arrange(glove,tx_buffer);
					int err = sendto(sock, tx_buffer, sizeof(tx_buffer), 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
					if (err < 0) {
						ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
						break;
					}
					int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, MSG_DONTWAIT, (struct sockaddr *)&source_addr, &socklen);

				}
				
			}
			break;
		}

		if (sock != -1) {
			ESP_LOGE(TAG, "Shutting down socket and restarting...");
			shutdown(sock, 0);
			close(sock);
		}
	}
	vTaskDelete(NULL);

} 
/**
 * @brief      Set up multiplexer and create a event to the task read a channel
 *
 * @param      pvParameters  The pv parameters
 */
static void sync_task(void *pvParameters)
{
	uint8_t addr[3]={1,0,2}; 
	mux_selector_config();
	vTaskDelay(pdMS_TO_TICKS(1000));
	xEventGroupSetBits(xEventGroup,STARTAQ);

	while(1){

		for (int i = 0; i < 2; ++i)
		{
			gpio_set_level(pinA,(addr[i]&2) >> 1);
			gpio_set_level(pinB, addr[i] & 1);
			xEventGroupSync(xEventGroup,(long unsigned int)1<<i,SYNCHRONIZED,xDelay);    
		}
		
		vTaskDelay(sample_time);
		xEventGroupSync(xEventGroup, STOPAQ,RESTARTAQ ,xDelay);
	}
	vTaskDelete(NULL);  
}

void app_main(void)
{
	xEventGroup = xEventGroupCreate();
	glove = (Glove *)malloc(sizeof(Glove));
	
	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	ESP_ERROR_CHECK(example_connect());

	adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));

	i2c_master_init(MASTER_0,SDA1,SCL1);
	i2c_master_init(MASTER_1,SDA2,SCL2);
	
	setup_sensors();
	initialization(glove);
	calibration(glove);
	adc_config();	

	xTaskCreate(udp_server_task, "udp_server_task", 16384, (void*)AF_INET, 5, &xTaskUDP);//!Task instance for udp comunication
	xTaskCreate(i2c_task_reference_frame , "i2c_task_reference_frame", 4096, (void *)1, 10, &xTaskREF); //!Task instance for I2C BUS read.
	xTaskCreate(i2c_task0 , "i2c_test_task_0", 4096, (void *)0, 20, &xTaskI2C0); //!Task instance for I2C BUS read.
	xTaskCreate(sync_task , "sync_task", 2048, (void *)0, 20, &xTaskSYNCH); //!Task instance for I2C BUS read.
	xTaskCreate(disp_buf  , "disp_buf", 4096, (void *)UDP, 20, &xTaskDISP);//!< Task instance for prety print I2C BUS on esp32 monitor on PC.

}