/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>

#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_err.h"
#include "protocol_examples_common.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "driver/i2c.h"

#define PORT CONFIG_EXAMPLE_PORT


/**
 * TEST CODE BRIEF
 *
 * This example will show you how to use I2C module by running two tasks on i2c bus:
 *
 * - read external i2c sensor, here we use a ADXL345 sensor for instance.
 * - Use one I2C port(master mode) to read or write the other I2C port(slave mode) on one ESP8266 chip.
 *
 * Pin assignment:
 *
 * - master:
 *    GPIO14 is assigned as the data signal of i2c master port
 *    GPIO2 is assigned as the clock signal of i2c master port
 *
 * Connection:
 *
 * - connect sda/scl of sensor with GPIO14/GPIO2
 * - no need to add external pull-up resistors, driver will enable internal pull-up resistors.
 *
 * Test items:
 *
 * - read the sensor data, if connected.
 */

#define I2C_EXAMPLE_MASTER_SCL_IO           2                /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO           14               /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM              I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0                /*!< I2C master do not need buffer */

/* CONTROL ADDRESS AND STATUS*/
#define ADXL345_SENSOR_ADDR_ref               0x1D            /*!< slave address for ADXL345 sensor */
#define ADXL345_SENSOR_ADDR_pos               0x53            /*!< slave address for ADXL345 sensor */

#define WRITE_BIT                           I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                            I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                        0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x0              /*!< I2C ack value */
#define NACK_VAL                            0x1              /*!< I2C nack value */
#define LAST_NACK_VAL                       0x2              /*!< I2C last_nack value */

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
QueueHandle_t buffer_queue; 


/* i2c_master_init*
    * @brief i2c master initialization
    */
static esp_err_t i2c_master_init(){
  int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
            //conf.sda_pullup_en = 1;
  conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
            //conf.scl_pullup_en = 1;
      conf.clk_stretch_tick = 300; // 300 ticks, Clock stretch is about 210us, you can make changes according to the actual situation.
      ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
      ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
      return ESP_OK;
    }

/* i2c_master_ADXL345_write*
 * @brief code to write ADXL345
 *
 * 1. send data
 * ___________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | write data_len byte + ack  | stop |
 * --------|---------------------------|-------------------------|----------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to send
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
    static esp_err_t i2c_master_ADXL345_write(i2c_port_t i2c_num ,uint8_t device ,uint8_t reg_address, uint8_t *data, size_t data_len)
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

/* i2c_master_ADXL345_read*
 * @brief test code to read ADXL345
 *
 * 1. send reg address
 * ______________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | stop |
 * --------|---------------------------|-------------------------|------|
 *
 * 2. read data
 * ___________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | read data_len byte + ack(last nack)  | stop |
 * --------|---------------------------|--------------------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param device to read
 * @param reg_address slave reg address
 * @param data data to read
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
    static esp_err_t i2c_master_ADXL345_read(i2c_port_t i2c_num,uint8_t device ,uint8_t reg_address, uint8_t *data, size_t data_len)
    {
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


static esp_err_t i2c_master_ADXL345_init(i2c_port_t i2c_num, uint8_t device)
{
      uint8_t cmd_data;
      vTaskDelay(100 / portTICK_RATE_MS);
  cmd_data = 0x0B;    // ±16g, 13-BIT MODE
  ESP_ERROR_CHECK(i2c_master_ADXL345_write(i2c_num, device,DATA_FORMAT, &cmd_data, 1));
  cmd_data = 0x08;    // START MEASUREMENT
  ESP_ERROR_CHECK(i2c_master_ADXL345_write(i2c_num, device,POWER_CTL, &cmd_data, 1));
  cmd_data = 0x80;    // ENABLE DATA_READY INTERRUPT.
  ESP_ERROR_CHECK(i2c_master_ADXL345_write(i2c_num, device,INT_ENABLE, &cmd_data, 1));

  return ESP_OK;
}


static void i2c_task()
{
  int ret,ret1;
  int16_t buffer[6];
  uint8_t sensor_ref[6], sensor_pos[6];

  i2c_master_init();
  i2c_master_ADXL345_init(I2C_EXAMPLE_MASTER_NUM, ADXL345_SENSOR_ADDR_ref);
  i2c_master_ADXL345_init(I2C_EXAMPLE_MASTER_NUM, ADXL345_SENSOR_ADDR_pos);

  memset(buffer, 0, 6);
  memset(sensor_pos, 0, 6);
  memset(sensor_ref, 0, 6);
  
  while(1){
    ret = i2c_master_ADXL345_read(I2C_EXAMPLE_MASTER_NUM, ADXL345_SENSOR_ADDR_ref, DATAX0, sensor_ref,6);
    ret1 = i2c_master_ADXL345_read(I2C_EXAMPLE_MASTER_NUM, ADXL345_SENSOR_ADDR_pos, DATAX0, sensor_pos,6);
    if ((ret == ESP_OK) && (ret1 == ESP_OK)) {

      buffer[0] = (int16_t)((sensor_ref[XMSB] << 8) | sensor_ref[XLSB]);
      buffer[1] = (int16_t)((sensor_ref[YMSB] << 8) | sensor_ref[YLSB]);
      buffer[2] = (int16_t)((sensor_ref[ZMSB] << 8) | sensor_ref[ZLSB]);
      buffer[3] = (int16_t)((sensor_pos[XMSB] << 8) | sensor_pos[XLSB]);
      buffer[4] = (int16_t)((sensor_pos[YMSB] << 8) | sensor_pos[YLSB]);
      buffer[5] = (int16_t)((sensor_pos[ZMSB] << 8) | sensor_pos[ZLSB]);

      xQueueSend(buffer_queue, &buffer, pdMS_TO_TICKS(1));
      
    } 

    else {
      ESP_LOGE(TAG, "No ack, sensor %s not connected...skip...\n", ret == ESP_OK ? 
        "Posição" : ret1 == ESP_OK ? "Referência" : "None");
    }

    vTaskDelay(10/portTICK_RATE_MS); 
  }

  i2c_driver_delete(I2C_EXAMPLE_MASTER_NUM);
}

static void udp_server_task(void *pvParameters)
{
  char rx_buffer[128]={'0'};
  char tx_buffer_msg[128]={'0'};
  int len_to_send=0;
  int16_t tx_buffer[6];
  char addr_str[128];
  int addr_family;
  int ip_protocol;


  struct sockaddr_in destAddr;
  destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  destAddr.sin_family = AF_INET;
  destAddr.sin_port = htons(PORT);
  addr_family = AF_INET;
  ip_protocol = IPPROTO_IP;
  inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);

  while(1){
    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
      ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
      break;
    }
    ESP_LOGI(TAG, "Socket created");

    int err = bind(sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
    if (err < 0) {
      ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    }

    ESP_LOGI(TAG, "Socket binded");
    ESP_LOGI(TAG, "Waiting for address");

    struct sockaddr_in sourceAddr;

    socklen_t socklen = sizeof(sourceAddr);
    int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&sourceAddr, &socklen);

// Error occured during receiving
    if (len < 0) {
      ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
      break;
    }
// Data received
    else {
// Get the sender's ip address as string

      inet_ntoa_r(((struct sockaddr_in *)&sourceAddr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);

      while (1) {

        if(xQueueReceive(buffer_queue, &tx_buffer, pdMS_TO_TICKS(10))==true){

          len_to_send = sprintf(tx_buffer_msg,"%d;%d;%d;%d;%d;%d\r\n",
            tx_buffer[0],
            tx_buffer[1],
            tx_buffer[2],
            tx_buffer[3],
            tx_buffer[4],
            tx_buffer[5]);

          int err = sendto(sock, tx_buffer_msg, len_to_send, 0, (struct sockaddr *)&sourceAddr, sizeof(sourceAddr));
          ESP_LOGI(TAG,"%s",tx_buffer_msg);
          if (err < 0) {
            ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
            break;
          }

          ESP_LOGI(TAG,"%s",tx_buffer_msg);
          vTaskDelay(20 / portTICK_RATE_MS);  
        }
        else{
          ESP_LOGI(TAG,"TIMEOUT_ERROR:");
        }
      }


      if (sock != -1) {
        ESP_LOGE(TAG, "Shutting down socket...");
        shutdown(sock, 0);
        close(sock);
      }
    }
  }

  vTaskDelete(NULL);
}
void app_main()
{

buffer_queue = xQueueCreate(4, 6*sizeof(int16_t));//Cria a queue *buffer* com 10 slots de 4 Bytes

ESP_ERROR_CHECK(nvs_flash_init());
ESP_ERROR_CHECK(esp_netif_init());
ESP_ERROR_CHECK(esp_event_loop_create_default());

ESP_ERROR_CHECK(example_connect());

xTaskCreate(i2c_task, "udp_server", 4096, NULL, 5, NULL);
xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);

}

