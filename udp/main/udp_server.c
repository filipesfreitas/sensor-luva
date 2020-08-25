/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_MASTER_SCL_IO 19               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 18               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM 0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 400000        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define ADXL345_SLAVE_ADDRref  0X53   /*!< slave address for ADXL345 sensor */
#define ADXL345_SLAVE_ADDRpos1 0X1D   /*!< slave address for ADXL345 sensor */
#define ADXL345_SLAVE_ADDRpos2 0X53   /*!< slave address for ADXL345 sensor */
#define ADXL345_SLAVE_ADDRpos3 0X1D   /*!< slave address for ADXL345 sensor */
#define ADXL345_SLAVE_ADDRpos4 0X53   /*!< slave address for ADXL345 sensor */
#define ADXL345_SLAVE_ADDRpos5 0X1D   /*!< slave address for ADXL345 sensor */
#define ADXL345_SLAVE_ADDRpos6 0X53   /*!< slave address for ADXL345 sensor */
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */
#define LAST_NACK_VAL 0x2

/* Map of device registers*/
#define THRESH_TAP  0x1D        /*Tap threshold*/
#define OFSX    0x1E            /*X-axis offset*/
#define OFSY    0x1F            /*Y-axis offset*/
#define OFSZ    0x20            /*Z-axis offset*/
#define DUR 0x21                /*Tap duration*/
#define Latent  0x22            /*Tap latency*/
#define Window  0x23            /*Tap window*/
#define THRESH_ACT  0x24        /*Activity threshold*/
#define THRESH_INACT    0x25    /*Inactivity threshold*/
#define TIME_INACT  0x26        /*Inactivity time*/
#define ACT_INACT_CTL   0x27    /*Axis enable control for activity and inactivity detection*/
#define THRESH_FF   0x28        /*Free-fall threshold*/
#define TIME_FF 0x29            /*Free-fall time*/
#define TAP_AXES    0x2A        /*Axis control for single tap/double tap*/
#define ACT_TAP_STATUS  0x2B    /*Source of single tap/double tap*/
#define BW_RATE 0x2C            /*Data rate and power mode control*/
#define POWER_CTL   0x2D        /*Power-saving features control*/
#define INT_ENABLE  0x2E        /*Interrupt enable control*/
#define INT_MAP 0x2F            /*Interrupt mapping control*/
#define INT_SOURCE  0x30        /*Source of interrupts*/
#define DATA_FORMAT 0x31        /*Data format control*/
#define DATAX0  0x32            /*X-Axis Data 0 LSB*/
#define DATAX1  0x33            /*X-Axis Data 1 MSB*/
#define DATAY0  0x34            /*Y-Axis Data 0 LSB*/
#define DATAY1  0x35            /*Y-Axis Data 1 MSB*/
#define DATAZ0  0x36            /*Z-Axis Data 0 LSB*/
#define DATAZ1  0x37            /*Z-Axis Data 1 MSB*/
#define FIFO_CTL    0x38        /*FIFO control*/
#define FIFO_STATUS 0x39        /*FIFO status*/

#define XLSB 0 /* least significant byte of X' accelleration*/
#define XMSB 1 /* most significant byte of X' accelleration*/
#define YLSB 2 /* least significant byte of Y' accelleration*/ 
#define YMSB 3 /* most significant byte of Y' accelleration*/
#define ZLSB 4 /* least significant byte of Z' accelleration*/
#define ZMSB 5 /* most significant byte of Z' accelleration*/

static const char *TAG = "example";
#define PORT CONFIG_EXAMPLE_PORT

SemaphoreHandle_t print_mux = NULL;
QueueHandle_t buffer_queue; 
static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num,uint8_t device ,uint8_t reg_address, uint8_t *data, size_t data_len)
{
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, device << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 100/ portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  if (ret != ESP_OK) {
    return ret;
  }

  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, device << 1 | READ_BIT, ACK_CHECK_EN);
  i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 100 / portTICK_RATE_MS);
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


static esp_err_t i2c_master_ADXL345_init(uint8_t device)
{
  uint8_t cmd_data;
  vTaskDelay(100 / portTICK_RATE_MS);
  cmd_data = 0x0B;    // ±16g, 13-BIT MODE
  ESP_ERROR_CHECK(i2c_master_write_slave(I2C_MASTER_NUM, device,DATA_FORMAT, &cmd_data, 1));
  cmd_data = 0x08;    // START MEASUREMENT
  ESP_ERROR_CHECK(i2c_master_write_slave(I2C_MASTER_NUM, device,POWER_CTL, &cmd_data, 1));

  return ESP_OK;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
  int i2c_master_port = I2C_MASTER_NUM;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_MASTER_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = I2C_MASTER_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  i2c_param_config(i2c_master_port, &conf);
  return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief test function to show buffer
 */
static void disp_buf(uint8_t *buf, int len)
{
  int i;
  for (i = 0; i < len; i++) {
    printf("%02x ", buf[i]);
  }
  printf("\n");
}

static void i2c_test_task(void *arg)
{
  int ret;
  uint32_t task_idx = (uint32_t)arg;
  uint8_t sensor[6];
  uint8_t device = 0x53;
  int cnt = 0;
  int16_t buffer[4];

  memset(buffer, 0, 8);
  memset(sensor, 0, 6);

  switch( (int)arg){
    case 0: ESP_ERROR_CHECK(i2c_master_ADXL345_init(ADXL345_SLAVE_ADDRref)); 
    break;

    case 1: ESP_ERROR_CHECK(i2c_master_ADXL345_init(ADXL345_SLAVE_ADDRpos1));
    break;

    case 2: ESP_ERROR_CHECK(i2c_master_ADXL345_init(ADXL345_SLAVE_ADDRpos2));
    break;

    case 3: ESP_ERROR_CHECK(i2c_master_ADXL345_init(ADXL345_SLAVE_ADDRpos3));
    break;

    case 4: ESP_ERROR_CHECK(i2c_master_ADXL345_init(ADXL345_SLAVE_ADDRpos4));
    break;

    case 5: ESP_ERROR_CHECK(i2c_master_ADXL345_init(ADXL345_SLAVE_ADDRpos5));
    break;
    case 6: ESP_ERROR_CHECK(i2c_master_ADXL345_init(ADXL345_SLAVE_ADDRpos6));
    break;
  }
  
  while (1) {

    ret = i2c_master_read_slave(I2C_MASTER_NUM, device,DATAX1,sensor, 6);
    xSemaphoreTake(print_mux, portMAX_DELAY);
    if (ret == ESP_ERR_TIMEOUT) {
      ESP_LOGE(TAG, "I2C Timeout");
    } else if (ret == ESP_OK) {
      buffer[0] = (int16_t)((sensor[XMSB] << 8) | sensor[XLSB]);
      buffer[1] = (int16_t)((sensor[YMSB] << 8) | sensor[YLSB]);
      buffer[2] = (int16_t)((sensor[ZMSB] << 8) | sensor[ZLSB]);
      buffer[3] = (int) arg; 
      xQueueSend(buffer_queue, &buffer, pdMS_TO_TICKS(1));
      //printf("TASK[%d]  MASTER READ SENSOR( adxl345 )\tdata: ", task_idx);
      //disp_buf(sensor,6);

    } else {
      ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
    }
    xSemaphoreGive(print_mux);
    vTaskDelay(20/ portTICK_RATE_MS);
  }
  vSemaphoreDelete(print_mux);
  vTaskDelete(NULL);
}

static void udp_server_task(void *pvParameters)
{

  char addr_str[128];
  int addr_family = (int)pvParameters;
  int ip_protocol = 0;
  struct sockaddr_in6 dest_addr;

  char rx_buffer[128]={'0'};
  char tx_buffer_msg[128]={'0'};
  int len_to_send=0;
  int16_t tx_buffer[7];

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
    xSemaphoreTake(print_mux, portMAX_DELAY);
    if(xQueueReceive(buffer_queue, &tx_buffer, pdMS_TO_TICKS(10))==true){

      len_to_send = sprintf(tx_buffer_msg,"%d;%d;%d;%d\r\n",
        tx_buffer[0],
        tx_buffer[1],
        tx_buffer[2],
        tx_buffer[3]);

      int err = sendto(sock, tx_buffer_msg, len_to_send, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
      ESP_LOGI(TAG,"%s",tx_buffer_msg);
      if (err < 0) {
        ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
        break;
      }

      ESP_LOGI(TAG,"%s",tx_buffer_msg);
      vTaskDelay(20 / portTICK_RATE_MS); 
      if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        break;
      }


    }
    xSemaphoreGive(print_mux);
  }

}

if (sock != -1) {
  ESP_LOGE(TAG, "Shutting down socket and restarting...");
  shutdown(sock, 0);
  close(sock);
}
}
vTaskDelete(NULL);  
vSemaphoreDelete(print_mux);
} 

void app_main(void)
{
    print_mux = xSemaphoreCreateMutex();
    buffer_queue = xQueueCreate(4, 7*sizeof(int16_t));//Cria a queue *buffer* com  slots de 4 Bytes
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());
    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);
    xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL);
    xTaskCreate(i2c_test_task, "i2c_test_task_1", 1024 * 2, (void *)1, 10, NULL);
  }