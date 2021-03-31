
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
#include "position_process.h"
#include <string.h>
#include <sys/param.h>

#define I2C_MASTER_FREQ_HZ 400000        /* I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0      /* I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0      /* I2C master doesn't need buffer */

#define ADXL345_SLAVE_ADDR0   0x53   /* slave address for ADXL345 sensor */
#define ADXL345_SLAVE_ADDR1   0x1d   /* slave address for ADXL345 sensor */

#define WRITE_BIT               I2C_MASTER_WRITE /* I2C master write */
#define READ_BIT                I2C_MASTER_READ  /* I2C master read */
#define ACK_CHECK_EN            0x1              /* I2C master will check ack from slave*/
#define ACK_CHECK_DIS           0x0              /* I2C master will not check ack from slave */
#define ACK_VAL                 0x0              /* I2C ack value */
#define NACK_VAL                0x1              /* I2C nack value */
#define LAST_NACK_VAL           0x2 			 /* I2C nack value */

#define ACCEL_CONFIG_  0x1C
#define GYRO_CONFIG_   0x1B
#define ACCEL_CONFIG2_ 0x1D
#define CLKSEL_PLL_ 	 0x01
#define I2C_MST_CTRL_  0x24
#define I2C_MST_CLK_   0x0D
#define PWR_MGMNT_1_   0x6B
#define H_RESET_ 			 0x80

#define first_byte  0x3B  /* X-Axis ACCEL 0 LSB*/

#define XLSB 0 /* least significant byte of X' accelleration*/
#define XMSB 1 /* most significant byte of X' accelleration*/
#define YLSB 2 /* least significant byte of Y' accelleration*/ 
#define YMSB 3 /* most significant byte of Y' accelleration*/
#define ZLSB 4 /* least significant byte of Z' accelleration*/
#define ZMSB 5 /* most significant byte of Z' accelleration*/

#define MPU_ADDRESS_ref 0x68
#define MPU_ADDRESS_pos 0x69 

#define PORT CONFIG_EXAMPLE_PORT

#define PORT0ADX  ( 1UL << 0UL )  /* Event bit 0, read two sensors at the port0 i2c */
#define PORT1ADX 	( 1UL << 1UL ) 	/* Event bit 1, read two sensors at the port1 i2c */
#define UDP  			( 1UL << 2UL )  /* Event bit 2, UDP server */

#define ADC1      ( 1UL << 3UL )  /* ADC1 Event bit 3*/

#define ALLSYNCH  PORT1ADX | UDP/* check all samples were readed*/
#define SYNCHDATA PORT1ADX 

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static const char *TAG = "TAG_ESP";

EventGroupHandle_t xEventGroup;

QueueHandle_t buffer_queue; 

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_0;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;


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

static esp_err_t i2c_imu_setup(i2c_port_t MASTER_NUMBER,uint8_t device)
{
	uint8_t cmd_data;
	vTaskDelay(100 / portTICK_RATE_MS);

	cmd_data = CLKSEL_PLL_;    // gyroscope clock selector
	ESP_ERROR_CHECK(i2c_master_write_slave(MASTER_NUMBER, device,PWR_MGMNT_1_, &cmd_data, 1));
	cmd_data = 0x00;    // gyroscope clock selector
	ESP_ERROR_CHECK(i2c_master_write_slave(MASTER_NUMBER, device,PWR_MGMNT_1_, &cmd_data, 1));
	

	cmd_data = I2C_MST_CLK_;// Master 400khz
	ESP_ERROR_CHECK(i2c_master_write_slave(MASTER_NUMBER, device,I2C_MST_CTRL_, &cmd_data, 1));
	
	/*cmd_data = 0X06;    // ACCEL RANGE
	ESP_ERROR_CHECK(i2c_master_write_slave(MASTER_NUMBER, device,ACCEL_CONFIG_, &cmd_data, 1));
	cmd_data = 0X06;    // GYRO RANGE
	ESP_ERROR_CHECK(i2c_master_write_slave(MASTER_NUMBER, device,GYRO_CONFIG_, &cmd_data, 1));

*/
	/*cmd_data = 0x0B;    // ±16g, 13-BIT MODE
	ESP_ERROR_CHECK(i2c_master_write_slave(MASTER_NUMBER, device,DATA_FORMAT, &cmd_data, 1));
	cmd_data = 0x08;    // START MEASUREMENT
	ESP_ERROR_CHECK(i2c_master_write_slave(MASTER_NUMBER, device,POWER_CTL, &cmd_data, 1));
*/

	return ESP_OK;
}

static esp_err_t i2c_master_init(i2c_port_t MASTER_NUMBER, int sda, int scl)
{

	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = sda;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num = scl;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
	i2c_param_config(MASTER_NUMBER, &conf);
	return i2c_driver_install(MASTER_NUMBER, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


static void disp_buf(void * pvParameters)
{
	int len_to_send=0;
	int16_t tx_buffer[7];
	char tx_buffer_msg[128]={'0'};
	int i=1;
	while(1){
		xEventGroupWaitBits(xEventGroup, PORT0ADX | PORT1ADX, pdFALSE,pdTRUE,(TickType_t)1);
		i =1;
		do{
			if(xQueueReceive(buffer_queue, &tx_buffer, portMAX_DELAY))
			{
				len_to_send = sprintf(tx_buffer_msg,"%d;%d;%d;%d;%d;%d;%d\r\n",
					tx_buffer[0],
					tx_buffer[1],
					tx_buffer[2],
					tx_buffer[3],
					tx_buffer[4],
					tx_buffer[5],
					tx_buffer[6]);

				ESP_LOGI(TAG,"%s\n",tx_buffer_msg);
			}

		}while(i++ == 2);
		xEventGroupSetBits(xEventGroup,(EventBits_t) pvParameters);
	}
}


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

int16_t adc_read(){
	//Check if Two Point or Vref are burned into eFuse
	check_efuse();
	//Configure ADC
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(channel, atten);		

  adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
  print_char_val_type(val_type);
	//Continuously sample ADC1

		uint32_t adc_reading = 0;
		//Multisampling
		for (int i = 0; i < NO_OF_SAMPLES; i++) {
			adc_reading += adc1_get_raw((adc1_channel_t)channel);
		}
		adc_reading /= NO_OF_SAMPLES;

		//Convert adc_reading to voltage in mV
		int16_t voltage = (int16_t) esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
		return voltage;
	
}
static void i2c_task(void *pvParameters)
{
	int ret=ESP_OK,ret1=ESP_OK;

	uint8_t sensor[14],sensor2[14];

	i2c_port_t master_num;

	int sda;
	int scl;

	int16_t buffer[15];
	esp_err_t error_setting[2]={ESP_OK,ESP_OK}; 

	memset(sensor,0,14);
	memset(sensor2,0,14);
	memset(buffer,0,30);

	switch( (int) pvParameters){
		case PORT0ADX: 
		master_num = 0;
		sda=22;
		scl=23;
		i2c_master_init(master_num,sda,scl);
		//error_setting[0] =  i2c_master_imu_setup(master_num,ADXL345_SLAVE_ADDR0); 
		error_setting[1]  = i2c_imu_setup(master_num,ADXL345_SLAVE_ADDR1); 
		break;

		default:
		master_num = 1;
		sda=18;
		scl=19;
		i2c_master_init(master_num,sda,scl);
		error_setting[0] =  i2c_imu_setup(master_num,MPU_ADDRESS_ref); 
		error_setting[1]  = i2c_imu_setup(master_num,MPU_ADDRESS_pos);
		break;

	}
	if(error_setting[0] || error_setting[1] != ESP_OK){
		ESP_LOGE(TAG, "Problem at master nº: %d\tNo ack, sensor %s not connected...skip...\n",master_num
			, error_setting[0] == ESP_OK ? "SENSOR 0X53" : error_setting[1] != ESP_OK ? "SENSOR 0X1D" : "None");
	}
	while (1) {
		memset(sensor,0,14);
		memset(sensor2,0,14);
		memset(buffer,0,30);
		if(master_num==0){

			ret1 = i2c_master_read_slave(master_num, MPU_ADDRESS_ref,first_byte,sensor2, 14);

		}
		else{
			ret = i2c_master_read_slave(master_num, MPU_ADDRESS_ref,first_byte,sensor, 14);
			ret1 = i2c_master_read_slave(master_num, MPU_ADDRESS_pos,first_byte,sensor2, 14);

		}


		if (ret == ESP_ERR_TIMEOUT || ret1 == ESP_ERR_TIMEOUT) {
			ESP_LOGE(TAG, "I2C Timeout");
		} 
		else if ((ret == ESP_OK) && (ret1 == ESP_OK)) {

			buffer[0] = (int16_t)((sensor[0] ) | sensor[1]);		/* ACCEL X*/
			buffer[1] = (int16_t)((sensor[2] ) | sensor[3]);		/* ACCEL Y*/
			buffer[2] = (int16_t)((sensor[4] ) | sensor[5]);		/* ACCEL Z*/
			buffer[3] = (int16_t)((sensor[6] ) | sensor[7]);		/* TEMP	  */
			buffer[4] = (int16_t)((sensor[8] ) | sensor[9]);		/* GIRO X */
			buffer[5] = (int16_t)((sensor[10]) | sensor[11]);		/* GIRO Y */
			buffer[6] = (int16_t)((sensor[12]) | sensor[13]);		/* GIRO Z */

			buffer[7]  = (int16_t)((sensor2[0] ) | sensor2[1]);	/* ACCEL X*/
			buffer[8]  = (int16_t)((sensor2[2] ) | sensor2[3]);	/* ACCEL Y*/
			buffer[9]  = (int16_t)((sensor2[4] ) | sensor2[5]);	/* ACCEL Z*/
			buffer[10] = (int16_t)((sensor2[6] ) | sensor2[7]);	/* TEMP	  */
			buffer[11] = (int16_t)((sensor2[8] ) | sensor2[9]);	/* GIRO X */
			buffer[12] = (int16_t)((sensor2[10]) | sensor2[11]);/* GIRO Y */
			buffer[13] = (int16_t)((sensor2[12]) | sensor2[13]);/* GIRO Z */
			buffer[14] = adc_read();
			if ((int) pvParameters == PORT1ADX)
			{
				xEventGroupWaitBits(xEventGroup, ADC1 , pdFALSE,pdTRUE,(TickType_t)1);
			}
			if(xQueueSend(buffer_queue, &buffer, portMAX_DELAY)!=pdPASS){
				ESP_LOGE(TAG, "failed to post on queue");
			}
		} 

		else {
			ESP_LOGE(TAG, "Problem at master nº: %d\tNo ack, sensor %s not connected...skip...\n",master_num
				, ret == ESP_OK ? "SENSOR 0X53" : ret1 != ESP_OK ? "SENSOR 0X1D" : "None");
		}
		xEventGroupSync(xEventGroup,(EventBits_t) pvParameters,ALLSYNCH,portMAX_DELAY);

	}
	vTaskDelete(NULL);
}

/*
* Read from Queue the data and send to Client.
* 
* 
*/
static void udp_server_task(void *pvParameters)
{
	char error_code_[512];
	char addr_str[128];
	int addr_family = (int)pvParameters;
	int ip_protocol = 0;
	struct sockaddr_in6 dest_addr;

	char rx_buffer[128]={'0'};
	int len_to_send=0;
	int16_t tx_buffer[15];
	char tx_buffer_msg[256]={'0'};
	int i;

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
		xEventGroupWaitBits(xEventGroup, PORT0ADX | PORT1ADX, pdFALSE,pdTRUE,(TickType_t)1);
		i =1;
		do{
			if(xQueueReceive(buffer_queue, &tx_buffer, pdMS_TO_TICKS(10))==true){
				len_to_send = sprintf(tx_buffer_msg,"%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d\r\n",
					tx_buffer[0],
					tx_buffer[1],
					tx_buffer[2],
					tx_buffer[3],
					tx_buffer[4],
					tx_buffer[5],
					tx_buffer[6],
					tx_buffer[7],
					tx_buffer[8],
					tx_buffer[9],
					tx_buffer[10],
					tx_buffer[11],
					tx_buffer[12],
					tx_buffer[13],
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


void app_main(void)
{

	xEventGroup = xEventGroupCreate();

	buffer_queue = xQueueCreate(6, 15*sizeof(int16_t));//Cria a queue *buffer* com 6 slots de 28 bytes
	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	ESP_ERROR_CHECK(example_connect());

	xTaskCreate(udp_server_task, "udp_server_task", 4096, (void*)AF_INET, 5, NULL);//!Task instance for udp comunication
	//xTaskCreate(i2c_task  , "i2c_test_task_0", 2048, (void *)PORT0ADX, 20, NULL); //!Task instance for I2C BUS read.
	xTaskCreate(i2c_task  , "i2c_task_1", 2048, (void *)PORT1ADX, 10, NULL);//!Task instance for I2C BUS read.
	//xTaskCreate(disp_buf  , "disp_buf", 2048, (void *)UDP, 20, NULL);//!< Task instance for prety print I2C BUS on esp32 monitor on PC.
}