#include "esp_adc_cal.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
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
#include <position_process.h>
#define PORT CONFIG_EXAMPLE_PORT

/* Configuration of I2C*/
#define I2C_MASTER_FREQ_HZ 1000000        /* I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0      /* I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0      /* I2C master doesn't need buffer */
#define SDA1 23
#define SCL1 22
#define SDA2 21 
#define SCL2 19
#define WRITE_BIT               I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN            0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS           0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                 0x0              /*!< I2C ack value */
#define NACK_VAL                0x1              /*!< I2C nack value */
#define LAST_NACK_VAL           0x2

/* Sensor especifications*/
	/*Default address*/
#define SLAVE1_ADD 0x68
#define SLAVE2_ADD 0x69

	/* STRAT-UP sequence setup*/
#define ACCEL_CONFIG 0X1C
#define CONFIG_device 0x1A
#define ENABLE_ACC_GIRO 0x38
#define GYRO_CONFIG 0x1B
#define PWR_MGMT_1 0x6B	
#define SMPLRT_DIV 0x19
#define START_READ_ADD 0x3B



/* Event group bit set*/
#define PORT0ADX  	( 1UL << 0UL )  /* Event bit 0, read two sensors at the port0 i2c */
#define PORT1ADX  	( 1UL << 1UL ) 	/* Event bit 1, read two sensors at the port1 i2c */
#define UDP  	  		( 1UL << 2UL )  /* Event bit 2, UDP server */
#define DISPBUFFER 	( 1UL << 3UL )  /* Display buffer for channel reading*/
#define MUX         ( 1UL << 4UL )
#define ALLSYNCH  PORT0ADX  | PORT1ADX | DISPBUFFER | MUX /* check all samples were readed*/

/* ADC configuration*/
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling


/*GPIO MUX SELECTION*/
#define pinA 5
#define pinB 17
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<pinA) | (1ULL<<pinB))

#define gyro_factor 16.4
#define acc_factor 2048
#define degre_conv 180/M_PI


/* Log details*/
static const char *TAG = "TAG_ESP";

/* Event group instantiation*/
EventGroupHandle_t xEventGroup;

/* Queue instntiation*/
QueueHandle_t buffer_queue; 
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;
static esp_adc_cal_characteristics_t *adc_chars;
/*
int64_t time_elaps;

time_elaps = esp_timer_get_time();
ESP_LOGE(TAG, "time elapsed:%lld", time_elaps-esp_timer_get_time());
*/



/* Master rad on I2C bus*/
static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num,uint8_t device ,uint8_t reg_address, uint8_t *data, size_t data_len)
{
	/* Master read from slave on I2C BUS.
	@param
	*/
	int ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, device << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, 10 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	if (ret != ESP_OK) {
		return ret;
	}

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, device << 1 | READ_BIT, ACK_CHECK_EN);
	i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, 10 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	return ret;
}

/*Master write on I2C bus*/
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num ,uint8_t device ,uint8_t reg_address, uint8_t *data, size_t data_len)
{
	int ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, device << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
	i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	return ret;
}

/* Set of registers to set after IMU has started up*/
static esp_err_t i2c_imu_setup(i2c_port_t MASTER_NUMBER,uint8_t device)
{
	uint8_t cmd_data;
	vTaskDelay(100 / portTICK_RATE_MS);

	/* ODR config and DLPF*/
	printf("\vdevice:%d \v ",device);
	cmd_data = 0X00;    // DEVICE CONFIG:  accel bw =  260 Hz delay =0; gyro bw=256Hz delay 0.98ms -> Fs = 8kHz.
	ESP_ERROR_CHECK(i2c_master_write_slave(MASTER_NUMBER, device,CONFIG_device, &cmd_data, 1));
	cmd_data = 0x19;    // ACCEL CONFIG: 16 g
	ESP_ERROR_CHECK(i2c_master_write_slave(MASTER_NUMBER, device,ACCEL_CONFIG, &cmd_data, 1));
	cmd_data = 0x19;  	// GYRO CONFIG: 2000 º/S
	ESP_ERROR_CHECK(i2c_master_write_slave(MASTER_NUMBER, device,GYRO_CONFIG, &cmd_data, 1));
	cmd_data = 0x0;    // SMPLRT_DIV : Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
	ESP_ERROR_CHECK(i2c_master_write_slave(MASTER_NUMBER, device,SMPLRT_DIV, &cmd_data, 1));

	/* Power Mode */
	cmd_data = 0x00;    // POWER 1
	ESP_ERROR_CHECK(i2c_master_write_slave(MASTER_NUMBER, device,PWR_MGMT_1, &cmd_data, 1));


	return ESP_OK;
}

/* Initialize master hardware and config. for comunnication*/
static esp_err_t i2c_master_init(i2c_port_t MASTER_NUMBER, int sda, int scl)
{

	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = sda;
	conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
	conf.scl_io_num = scl;
	conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
	conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
	i2c_param_config(MASTER_NUMBER, &conf);
	return i2c_driver_install(MASTER_NUMBER, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/* Helper for printing data sended through wifi*/
static void disp_buf(void * pvParameters)
{
	volatile int len_to_send=0;
	int16_t tx_buffer[7];
	char tx_buffer_msg[256]={'0'};
	int i=1;
	float orientation [10];
	while(1){
		xEventGroupWaitBits(xEventGroup, PORT0ADX, pdFALSE,pdTRUE,(TickType_t)1);
		i =1;
		do{
			if(xQueueReceive(buffer_queue, &tx_buffer, portMAX_DELAY))
			{
				orientation_estimation(tx_buffer,orientation);
				len_to_send = sprintf(tx_buffer_msg,"%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%d\r\n",
					orientation[0],
					orientation[1],
					orientation[2],
					orientation[3],
					orientation[4],
					orientation[5],
					orientation[6],
					orientation[7],
					orientation[8],
					orientation[9],
					tx_buffer[14]);

				//ESP_LOGI(TAG,"%s",tx_buffer_msg);
			}

		}while(i++ == 2);
		xEventGroupSetBits(xEventGroup,DISPBUFFER);
	}
}

/* Check calibration data table on ESP32*/
static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

/*Print Vref by efuse or default*/
static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

void adc_config(){

	//Check if Two Point or Vref are burned into eFuse
	check_efuse();
	//Configure ADC

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC_CHANNEL_0, atten);		
  adc1_config_channel_atten(ADC_CHANNEL_3, atten);	
  adc1_config_channel_atten(ADC_CHANNEL_4, atten);			
  adc1_config_channel_atten(ADC_CHANNEL_6, atten);		
  adc1_config_channel_atten(ADC_CHANNEL_7, atten);		

  adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
  print_char_val_type(val_type);
}

int16_t adc_read(int addr){
	uint32_t adc_reading = 0;
	static adc_channel_t channel;
	switch(addr){
		case 0:
		 channel = ADC_CHANNEL_0; 
		break;
		case 1:
		 channel = ADC_CHANNEL_3;
		break;
		case 2:
		 channel = ADC_CHANNEL_4;
		break;
		case 3:
		 channel = ADC_CHANNEL_6;
		break;
		case 4:
		 channel = ADC_CHANNEL_7;
		break;
	}

		adc_reading += adc1_get_raw((adc1_channel_t)channel);
	
		//Convert adc_reading to voltage in mV
		int16_t voltage = (int16_t) esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
		return voltage;	
}

void mux_selector_config(){
  gpio_config_t io_conf;
  //disable interrupt
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  //set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  //bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  //disable pull-down mode
  io_conf.pull_down_en = 0;
  //disable pull-up mode
  io_conf.pull_up_en = 0;
  //configure GPIO with the given settings
  gpio_config(&io_conf);
}

/* Task for reading I2C data*/
static void i2c_task(void *pvParameters)
{
	/* IMU CONFIG*/
	int ret, ret1;
	int16_t buffer[15];
	uint8_t sensor[14];
	uint8_t sensor2[14];

	i2c_port_t master_num=(int) pvParameters;
	esp_err_t error_setting[2]={ESP_OK,ESP_OK}; 

	/* Mux CONFIG*/
	uint8_t addr=0; /* aux variable for counting*/
  mux_selector_config();
  gpio_set_level(pinA, 0);/* Mux initial state*/
  gpio_set_level(pinB, 0);

	i2c_master_init(master_num,SDA1,SCL1);

	memset(sensor,0,14);
	memset(sensor2,0,14);
	memset(buffer,0,30);

	error_setting[0]  = i2c_imu_setup(master_num,SLAVE1_ADD); 
	error_setting[1]  = i2c_imu_setup(master_num,SLAVE2_ADD); 

	if(error_setting[0] || error_setting[1] != ESP_OK){
		ESP_LOGE(TAG, "Problem at master nº: %d\tNo ack, sensor %s not connected...skip...\n",master_num,
		error_setting[0] == ESP_OK ? "SENSOR 0x68" :
		error_setting[1] != ESP_OK ? "SENSOR 0x69" : "None");
	}
	while (1) {
		if(addr == 1) addr = 0; 		/* Check if addr overflow*/
  	gpio_set_level(pinA, (addr & 2) >> 1);
  	gpio_set_level(pinB, addr & 1);
		

		ret  = i2c_master_read_slave(master_num, SLAVE1_ADD,START_READ_ADD,sensor, 14);
		ret1 = i2c_master_read_slave(master_num, SLAVE2_ADD,START_READ_ADD,sensor2, 14);

		if (ret == ESP_ERR_TIMEOUT || ret1 == ESP_ERR_TIMEOUT) {
			ESP_LOGE(TAG, "I2C Timeout");
		} 
		else if (ret == ESP_OK && ret1 == ESP_OK) {
			/* Ref*/
			buffer[0]  = addr;
			buffer[1]  = master_num;
			buffer[2]  = (int16_t)((sensor[0]  << 8)  | sensor[1]);	    /* ACCEL X */
			buffer[3]  = (int16_t)((sensor[2]  << 8)  | sensor[3]);		  /* ACCEL y */
			buffer[4]  = (int16_t)((sensor[4]  << 8)  | sensor[5]);		  /* ACCEL z */
			buffer[5]  = (int16_t)((sensor[8]  << 8)  | sensor[9]);		  /* GIRO X  */
			buffer[6]  = (int16_t)((sensor[10] << 8)  | sensor[11]);	  /* GIRO Y  */
			buffer[7]  = (int16_t)((sensor[12] << 8)  | sensor[13]);	  /* GIRO Z  */
			buffer[8]  = (int16_t)((sensor2[0]  << 8)  | sensor2[1]);	    /* ACCEL X */
			buffer[9]  = (int16_t)((sensor2[2]  << 8)  | sensor2[3]);		  /* ACCEL Y */
			buffer[10] = (int16_t)((sensor2[4]  << 8)  | sensor2[5]);		  /* ACCEL Z */
			buffer[11] = (int16_t)((sensor2[8]  << 8)  | sensor2[9]);		  /* GIRO X  */
			buffer[12] = (int16_t)((sensor2[10] << 8)  | sensor2[11]);	  /* GIRO Y  */
			buffer[13] = (int16_t)((sensor2[12] << 8)  | sensor2[13]);	  /* GIRO Z  */
			
			/* Angulo através do potenciômetro*/
			buffer[14] = adc_read(addr);


			if(xQueueSend(buffer_queue, &buffer, portMAX_DELAY)!=pdPASS){
				ESP_LOGE(TAG, "failed to post on queue");
			}
		}
		memset(sensor,0,14);
		memset(sensor2,0,14);
		memset(buffer,0,30);
		addr ++; 
		xEventGroupSync(xEventGroup,PORT0ADX,DISPBUFFER,portMAX_DELAY);

	}
	vTaskDelete(NULL);
}

/* Read from Queue the data and send to Client.*/
static void udp_server_task(void *pvParameters)
{
	char addr_str[128];
	char error_code_[512];
	char rx_buffer[128]={'0'};
	char tx_buffer_msg[256]={'0'};

	float orientation[10];

	int addr_family = (int)pvParameters;
	int i;
	int ip_protocol = 0;
	int len_to_send=0;
	int16_t tx_buffer[15];

	struct sockaddr_in6 dest_addr;
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
		xEventGroupWaitBits(xEventGroup, PORT0ADX , pdFALSE,pdTRUE,(TickType_t)1);
		i =1;
		do{
			if(xQueueReceive(buffer_queue, &tx_buffer, pdMS_TO_TICKS(10))==true){
				orientation_estimation(tx_buffer,orientation);
				len_to_send = sprintf(tx_buffer_msg,"%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%d\r\n",
					orientation[0],
					orientation[1],
					orientation[2],
					orientation[3],
					orientation[4],
					orientation[5],
					orientation[6],
					orientation[7],
					orientation[8],
					orientation[9],
					tx_buffer[14]);

				int err = sendto(sock, tx_buffer_msg, len_to_send, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));

				if (err < 0) {
					ESP_LOGE(TAG, "Error occured during sending: errno %x\t%s", errno,esp_err_to_name_r(errno,error_code_,512));
					break;
				}
			}
		}while(i++ == 2);
		if (err<0)
		{
			break;
		}
		xEventGroupSetBits(xEventGroup,UDP);
	}
	}

	if (sock != -1) {
		ESP_LOGE(TAG, "Shutting down socket and restarting...");
		shutdown(sock, 0);
		close(sock);
	}
	}
	vTaskDelete(NULL);  
} 

/* Main function*/
void app_main(void)
{

	xEventGroup = xEventGroupCreate();

	buffer_queue = xQueueCreate(6, 15*sizeof(int16_t));//Cria a queue *buffer* com 6 slots de 28 bytes
	
	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	ESP_ERROR_CHECK(example_connect());

  gpio_set_level(pinA, 0);
  gpio_set_level(pinB, 0);	
  adc_config();	

  /*SET UP MCUS*/


	//xTaskCreate(udp_server_task, "udp_server_task", 4096, (void*)AF_INET, 5, NULL);//!Task instance for udp comunication
	xTaskCreate(i2c_task  , "i2c_test_task_0", 2048, (void *)PORT0ADX, 20, NULL); //!Task instance for I2C BUS read.
	//xTaskCreate(i2c_task  , "i2c_test_task_1", 2048, (void *)PORT1ADX, 20, NULL); //!Task instance for I2C BUS read.
	xTaskCreate(disp_buf  , "disp_buf", 4096, (void *)UDP, 20, NULL);//!< Task instance for prety print I2C BUS on esp32 monitor on PC.
}