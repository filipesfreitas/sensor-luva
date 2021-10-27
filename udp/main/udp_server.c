/** @file */ 
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "lwip/err.h"
#include <lwip/netdb.h>
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "nvs_flash.h"
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
#define RESTARTAQ			UDP|PRINTAQ
#define channels 2
/* Event group instantiation*/
EventGroupHandle_t xEventGroup;

/* Queue instntiation*/
QueueHandle_t buffer_queue; 

/*Glove instance*/
Glove* glove;

 	const TickType_t xDelay = 1500 / portTICK_PERIOD_MS;/*delay defined by period ms*/

/**
 * @brief      Display the buffer of the package when the device is in debugger mode.
 *
 * @param      pvParameters  Parameter for the task.
 */
static void disp_buf(void * pvParameters)
{

	char tx_buffer_msg[256]={'0'};
	while(1){
		buffer_arrange(glove, tx_buffer_msg);
		printf("\n%s",tx_buffer_msg);
		xEventGroupSync(xEventGroup,RESTARTAQ,STOPAQ,xDelay);

	}
}

/**
 * @brief      Task to read the values given by the IMUs and call the orientation estimate function to update the values of the hand posture stored in glove structure.
 *
 * @param      pvParameters  Number given to the task.
 */
static void i2c_task0(void *pvParameters)
{
	/* IMU CONFIG*/
	int finger=1;
	int ret, ret1;
	raw_data metacarpo, proximal;
	uint8_t sensor[14];
	uint8_t sensor2[14];

	int16_t buffer;
	i2c_port_t master_num=(int) pvParameters;

	memset(sensor,0,14);
	memset(sensor2,0,14);
	raw_data_zero(&metacarpo);
	raw_data_zero(&proximal);

	while (1) {	

		if (finger > 1) finger = 1;
		ret  = i2c_master_read_slave(master_num, SLAVE1_ADD,START_READ_ADD,sensor, 14);
		ret1 = i2c_master_read_slave(master_num, SLAVE2_ADD,START_READ_ADD,sensor2, 14);

		if (ret == ESP_ERR_TIMEOUT || ret1 == ESP_ERR_TIMEOUT) {
			ESP_LOGW(TAG,"\vProblem at master nº: %d\tSensor %s timed out...skip...\v",master_num,
				ret  != ESP_OK ? "SENSOR 0x68" :
				ret1 != ESP_OK ? "SENSOR 0x69" : "None");
		} 
		else if (ret == ESP_OK && ret1 == ESP_OK) {
			/* Ref*/
			metacarpo.finger	= finger;
			metacarpo.finger	= master_num;
			metacarpo.accelx	= (int16_t)((sensor[0]   << 8) | sensor[1]);	/* ACCEL X */
			metacarpo.accely	= (int16_t)((sensor[2]   << 8) | sensor[3]);	/* ACCEL y */
			metacarpo.accelz	= (int16_t)((sensor[4]   << 8) | sensor[5]);	/* ACCEL z */
			metacarpo.gyrox		= (int16_t)((sensor[8]   << 8) | sensor[9]);	/* GIRO X  */
			metacarpo.gyroy		= (int16_t)((sensor[10]  << 8) | sensor[11]); /* GIRO Y  */
			metacarpo.gyroz		= (int16_t)((sensor[12]  << 8) | sensor[13]); /* GIRO Z  */
			proximal.accelx		= (int16_t)((sensor2[0]  << 8) | sensor2[1]); /* ACCEL X */
			proximal.accely		= (int16_t)((sensor2[2]  << 8) | sensor2[3]);	/* ACCEL Y */
			proximal.accelz		= (int16_t)((sensor2[4]  << 8) | sensor2[5]);	/* ACCEL Z */
			proximal.gyrox		= (int16_t)((sensor2[8]  << 8) | sensor2[9]);	/* GIRO X  */
			proximal.gyroy		= (int16_t)((sensor2[10] << 8) | sensor2[11]);/* GIRO Y  */
			proximal.gyroz		= (int16_t)((sensor2[12] << 8) | sensor2[13]);/* GIRO Z  */
			
			/* Pressão através do potenciômetro*/
			buffer = adc_read(finger,adc_chars);
			glove -> fingers[finger].pressure = buffer/(Vinput - buffer)*R;
			orientation_estimation(metacarpo,proximal,glove,finger);
		}
		memset(sensor,0,14);
		memset(sensor2,0,14);
		raw_data_zero(&metacarpo);
		raw_data_zero(&proximal);
		finger++;

		xEventGroupSync(xEventGroup,PORT0ADX,SYNCHRONIZED,xDelay);
	}
	vTaskDelete(NULL);
}

/**
 * @brief      Take raw samples from the reference frame device.
 *
 * @param      pvParameters  Task parameters.
 */
static void i2c_task_reference_frame(void *pvParameters)
{
	
	int ret;
	raw_data ref;
	uint8_t sensor[14];

	i2c_port_t master_num = (int) pvParameters;

	memset(sensor,0,14);
	raw_data_zero(&ref);
	int time;

	while (1) {	

		time = esp_timer_get_time();


		if (channels==1)
		{
			ret  = i2c_master_read_slave(master_num, SLAVE1_ADD,START_READ_ADD,sensor, 14);

			if (ret == ESP_ERR_TIMEOUT) {
				ESP_LOGE(TAG, "I2C Timeout");
			} 
			else if (ret == ESP_OK) {
			/* Ref*/
				ref.master_num  = master_num;
				ref.finger  = 0;
			ref.accelx  = (int16_t)((sensor[0]   << 8)  | sensor[1]); /* ACCEL X */
			ref.accely  = (int16_t)((sensor[2]   << 8)  | sensor[3]); /* ACCEL y */
			ref.accelz  = (int16_t)((sensor[4]   << 8)  | sensor[5]); /* ACCEL z */
			ref.gyrox   = (int16_t)((sensor[8]   << 8)  | sensor[9]); /* GIRO X  */
			ref.gyroy   = (int16_t)((sensor[10]  << 8)  | sensor[11]);/* GIRO Y  */
			ref.gyroz   = (int16_t)((sensor[12]  << 8)  | sensor[13]);/* GIRO Z  */

				reference_frame_orientation(ref,glove);
			}
			memset(sensor,0,14);
			raw_data_zero(&ref);
		}
		printf("%lld\n", esp_timer_get_time()-time);

		xEventGroupSync(xEventGroup,PORT1ADX,SYNCHRONIZED,xDelay);
	}
}
/* Read from Queue the data and send to Client.*/
static void udp_server_task(void *pvParameters){
	char addr_str[128];
	char rx_buffer[128]={'0'};
	char tx_buffer_msg[256]={'0'};
	int addr_family = (int)pvParameters;
	int ip_protocol = 0;
	struct sockaddr_in6 dest_addr;

	int16_t tx_buffer[15];

	struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
	dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
	dest_addr_ip4->sin_family = AF_INET;
	dest_addr_ip4->sin_port = htons(PORT);

	ip_protocol = IPPROTO_IP;

	while(1){
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

		ESP_LOGI(TAG, "Waiting for data");
	struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
	socklen_t socklen = sizeof(source_addr);
	int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

	// Error occurred during receiving
	if (len < 0) {
		ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
		break;
	}
	// Data received
	else {

	// Get the sender's ip address as string
		if (source_addr.sin6_family == PF_INET) {
			inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
		} else if (source_addr.sin6_family == PF_INET6) {
			inet6_ntoa_r(source_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
		}

	rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
	ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
	ESP_LOGI(TAG, "%s", rx_buffer);

	while(1){

		xEventGroupWaitBits(xEventGroup, STOPAQ , pdFALSE,pdTRUE,xDelay);
		if(xQueueReceive(buffer_queue, &tx_buffer, pdMS_TO_TICKS(10))==true){

			buffer_arrange(glove,tx_buffer_msg);
			int err = sendto(sock, tx_buffer_msg, strlen(tx_buffer_msg), 0, (struct sockaddr *)&source_addr, sizeof(source_addr));

			if (err<0)
			{
				break;
			}
		}
		xEventGroupSetBits(xEventGroup,UDP);
	}

	if (sock != -1) {
		ESP_LOGE(TAG, "Shutting down socket and restarting...");
		shutdown(sock, 0);
		close(sock);
	}
}
}
vTaskDelete(NULL);  
} 

static void sync_task(void *pvParameters)
{
	uint8_t addr=0; /* aux variable for counting*/
	/* Mux CONFIG*/
	mux_selector_config();
	while(1){
		while(addr < channels) { 		/* Check if addr overflow*/

		gpio_set_level(pinA, (addr & 2) >> 1);
		gpio_set_level(pinB, addr & 1);
		addr ++;
		xEventGroupSync(xEventGroup,SYNCHRONIZED,DISPBUFFER,xDelay);

	}
	addr = 0;
	xEventGroupSync(xEventGroup, STOPAQ,RESTARTAQ ,xDelay);
	vTaskDelay(xDelay);
}
vTaskDelete(NULL);  
}

/* Main function*/
void app_main(void)
{
	xEventGroup = xEventGroupCreate();

	buffer_queue = xQueueCreate(6, sizeof(Glove));//Create buffer queue with 6 slots of glove structure
	glove = (Glove *)malloc(sizeof(Glove));
	
	/*
	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	ESP_ERROR_CHECK(example_connect());
	*/
	adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));

	i2c_master_init(MASTER_0,SDA1,SCL1);
	i2c_master_init(MASTER_1,SDA2,SCL2);

	initialization(glove);
	calibration(glove);
	adc_config();	
	
	//xTaskCreate(udp_server_task, "udp_server_task", 4096, (void*)AF_INET, 5, NULL);//!Task instance for udp comunication
	xTaskCreate(sync_task , "sync_task", 2048, (void *)0, 20, NULL); //!Task instance for I2C BUS read.
	xTaskCreate(i2c_task0 , "i2c_test_task_0", 4096, (void *)0, 20, NULL); //!Task instance for I2C BUS read.
	xTaskCreate(i2c_task_reference_frame , "i2c_task_reference_frame", 2048, (void *)1, 20, NULL); //!Task instance for I2C BUS read.
	xTaskCreate(disp_buf  , "disp_buf", 4096, (void *)UDP, 20, NULL);//!< Task instance for prety print I2C BUS on esp32 monitor on PC.
}