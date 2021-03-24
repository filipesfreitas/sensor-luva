#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "driver/i2c.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define I2C_MASTER_FREQ_HZ 400000        /** I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0      /** I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0      /** I2C master doesn't need buffer */

#define ADXL345_SLAVE_ADDR0   0x53   /** slave address for ADXL345 sensor */
#define ADXL345_SLAVE_ADDR1   0x1d   /** slave address for ADXL345 sensor */

#define WRITE_BIT               I2C_MASTER_WRITE /** I2C master write */
#define READ_BIT                I2C_MASTER_READ  /** I2C master read */
#define ACK_CHECK_EN            0x1              /** I2C master will check ack from slave*/
#define ACK_CHECK_DIS           0x0              /** I2C master will not check ack from slave */
#define ACK_VAL                 0x0              /** I2C ack value */
#define NACK_VAL                0x1              /** I2C nack value */
#define LAST_NACK_VAL           0x2

/* Map of device registers*/
#define THRESH_TAP  0x1D        /** Tap threshold*/
#define OFSX    0x1E            /** X-axis offset*/
#define OFSY    0x1F            /** Y-axis offset*/
#define OFSZ    0x20            /** Z-axis offset*/
#define DUR 0x21                /** Tap duration*/
#define Latent  0x22            /** Tap latency*/
#define Window  0x23            /** Tap window*/
#define THRESH_ACT  0x24        /** Activity threshold*/
#define THRESH_INACT    0x25    /** Inactivity threshold*/
#define TIME_INACT  0x26        /** Inactivity time*/
#define ACT_INACT_CTL   0x27    /** Axis enable control for activity and inactivity detection*/
#define THRESH_FF   0x28        /** Free-fall threshold*/
#define TIME_FF 0x29            /** Free-fall time*/
#define TAP_AXES    0x2A        /** Axis control for single tap/double tap*/
#define ACT_TAP_STATUS  0x2B    /** Source of single tap/double tap*/
#define BW_RATE 0x2C            /** Data rate and power mode control*/
#define POWER_CTL   0x2D        /** Power-saving features control*/
#define INT_ENABLE  0x2E        /** Interrupt enable control*/
#define INT_MAP 0x2F            /** Interrupt mapping control*/
#define INT_SOURCE  0x30        /** Source of interrupts*/
#define DATA_FORMAT 0x31        /** Data format control*/
#define DATAX0  0x32            /** X-Axis Data 0 LSB*/
#define DATAX1  0x33            /** X-Axis Data 1 MSB*/
#define DATAY0  0x34            /** Y-Axis Data 0 LSB*/
#define DATAY1  0x35            /** Y-Axis Data 1 MSB*/
#define DATAZ0  0x36            /** Z-Axis Data 0 LSB*/
#define DATAZ1  0x37            /** Z-Axis Data 1 MSB*/
#define FIFO_CTL    0x38        /** FIFO control*/
#define FIFO_STATUS 0x39        /** FIFO status*/

#define XLSB 0 /* least significant byte of X' accelleration*/
#define XMSB 1 /* most significant byte of X' accelleration*/
#define YLSB 2 /* least significant byte of Y' accelleration*/ 
#define YMSB 3 /* most significant byte of Y' accelleration*/
#define ZLSB 4 /* least significant byte of Z' accelleration*/
#define ZMSB 5 /* most significant byte of Z' accelleration*/

#define MPU_ADDRESS
#define MPU_reg1
#define MPU_reg2
#define MPU_reg3
#define MPU_reg4
#define MPU_reg5
#define MPU_reg6

#define PORT CONFIG_EXAMPLE_PORT

#define PORT0ADX    ( 1UL << 0UL )  /* Event bit 0, read two sensors at the port0 i2c */
#define PORT1ADX 	( 1UL << 1UL ) 	/* Event bit 1, read two sensors at the port1 i2c */
#define UDP  		( 1UL << 2UL )  /* Event bit 2, UDP server */

#define RESTART 	( 1UL << 3UL )  /* Restart the reading events for more samples*/
#define ADC1        ( 1UL << 4UL )  /* ADC1 POT READ*/
#define ADC2        ( 1UL << 5UL )  /* ADC2 POT READ*/
#define ADC3		( 1UL << 6UL )  /* ADC3 POT READ*/

#define ALLSYNCH  PORT1ADX | UDP /* check all samples were readed*/
#define TESTSYNCH ADC1|ADC2|ADC3	
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static const char *TAG = "example";

EventGroupHandle_t xEventGroup;

QueueHandle_t buffer_queue; 


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


static esp_err_t i2c_master_imu_setup(i2c_port_t MASTER_NUMBER,uint8_t device)
{
	uint8_t cmd_data;
	vTaskDelay(100 / portTICK_RATE_MS);
	cmd_data = 0x0B;    // ±16g, 13-BIT MODE
	ESP_ERROR_CHECK(i2c_master_write_slave(MASTER_NUMBER, device,DATA_FORMAT, &cmd_data, 1));
	cmd_data = 0x08;    // START MEASUREMENT
	ESP_ERROR_CHECK(i2c_master_write_slave(MASTER_NUMBER, device,POWER_CTL, &cmd_data, 1));

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
static void i2c_test_task(void *pvParameters)
{
	int ret=ESP_OK,ret1=ESP_OK;

	uint8_t sensor[6],sensor2[6];

	i2c_port_t master_num;

	int sda;
	int scl;

	int16_t buffer[7];
	esp_err_t error_setting[2]={ESP_OK,ESP_OK}; 
	memset(sensor, 0, 6);
	memset(sensor2, 0, 6);
	memset(buffer, 0, 14);

	switch( (int) pvParameters){
		case PORT0ADX: 
		master_num = 0;
		sda=18;
		scl=19;
		i2c_master_init(master_num,sda,scl);
		//error_setting[0] =  i2c_master_imu_setup(master_num,ADXL345_SLAVE_ADDR0); 
		error_setting[1]  = i2c_master_imu_setup(master_num,ADXL345_SLAVE_ADDR1); 
		break;

		default:
		master_num = 1;
		sda=22;
		scl=23;
		i2c_master_init(master_num,sda,scl);
		error_setting[0] =  i2c_master_imu_setup(master_num,ADXL345_SLAVE_ADDR0); 
		error_setting[1]  = i2c_master_imu_setup(master_num,ADXL345_SLAVE_ADDR1);
		break;

	}
	if(error_setting[0] || error_setting[1] != ESP_OK){
		ESP_LOGE(TAG, "Problem at master nº: %d\tNo ack, sensor %s not connected...skip...\n",master_num
			, error_setting[0] == ESP_OK ? "SENSOR 0X53" : error_setting != ESP_OK ? "SENSOR 0X1D" : "None");
	}
	while (1) {
		memset(sensor, 0, 6);
		memset(sensor2, 0, 6);
		memset(buffer, 0, 14);
		if(master_num==0){

			ret1 = i2c_master_read_slave(master_num, ADXL345_SLAVE_ADDR1,DATAX0,sensor2, 6);

		}
		else{
			ret = i2c_master_read_slave(master_num, ADXL345_SLAVE_ADDR0,DATAX0,sensor, 6);
			ret1 = i2c_master_read_slave(master_num, ADXL345_SLAVE_ADDR1,DATAX0,sensor2, 6);

		}


		if (ret == ESP_ERR_TIMEOUT || ret1 == ESP_ERR_TIMEOUT) {
			ESP_LOGE(TAG, "I2C Timeout");
		} 
		else if ((ret == ESP_OK) && (ret1 == ESP_OK)) {

			buffer[0] = (int16_t)((sensor[XMSB] << 8) | sensor[XLSB]);
			buffer[1] = (int16_t)((sensor[YMSB] << 8) | sensor[YLSB]);
			buffer[2] = (int16_t)((sensor[ZMSB] << 8) | sensor[ZLSB]);
			buffer[3] = (int16_t)((sensor2[XMSB] << 8) | sensor2[XLSB]);
			buffer[4] = (int16_t)((sensor2[YMSB] << 8) | sensor2[YLSB]);
			buffer[5] = (int16_t)((sensor2[ZMSB] << 8) | sensor2[ZLSB]);
			buffer[6] = (int16_t) pvParameters;

			if ((int) pvParameters == PORT1ADX)
			{
				xEventGroupWaitBits(xEventGroup, PORT0ADX , pdFALSE,pdTRUE,(TickType_t)1);

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

/**
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
	int16_t tx_buffer[7];
	char tx_buffer_msg[128]={'0'};
	int i;

	if (addr_family == AF_INET) {
		struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
		dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
		dest_addr_ip4->sin_family = AF_INET;
		dest_addr_ip4->sin_port = htons(PORT);
		ip_protocol = IPPROTO_IP;
	} else if (addr_family == AF_INET6) {
		bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
		dest_addr.sin6_family = AF_INET6;
		dest_addr.sin6_port = htons(PORT);
		ip_protocol = IPPROTO_IPV6;
	}
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
				len_to_send = sprintf(tx_buffer_msg,"%d;%d;%d;%d;%d;%d;%d\r\n",
					tx_buffer[0],
					tx_buffer[1],
					tx_buffer[2],
					tx_buffer[3],
					tx_buffer[4],
					tx_buffer[5],
					tx_buffer[6]);

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

	buffer_queue = xQueueCreate(6, 7*sizeof(int16_t));//Cria a queue *buffer* com 4 slots de 14 Bytes
	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	ESP_ERROR_CHECK(example_connect());

	xTaskCreate(udp_server_task, "udp_server_task", 4096, (void*)AF_INET, 1, NULL);//!Task instance for udp comunication
	xTaskCreate(i2c_test_task  , "i2c_test_task_0", 2048, (void *)PORT0ADX, 20, NULL); //!Task instance for I2C BUS read.
	xTaskCreate(i2c_test_task  , "i2c_test_task_1", 2048, (void *)PORT1ADX, 20, NULL);//!Task instance for I2C BUS read.
	//xTaskCreate(disp_buf  , "i2c_test_task_1", 2048, (void *)UDP, 20, NULL);//!< Task instance for prety print I2C BUS on esp32 monitor on PC.

}