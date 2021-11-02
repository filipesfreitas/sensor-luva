#include <i2c_handler.h>
#include <misc.h>
#include "esp_log.h"

static const char *TAG = "I2C_HANDLER";

/* Master rad on I2C bus*/
esp_err_t i2c_master_read_slave(i2c_port_t i2c_num,uint8_t device ,uint8_t reg_address, uint8_t *data, size_t data_len)
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
esp_err_t i2c_master_write_slave(i2c_port_t i2c_num ,uint8_t device ,uint8_t reg_address, uint8_t *data, size_t data_len)
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
esp_err_t i2c_imu_setup(i2c_port_t MASTER_NUMBER,uint8_t device)
{
	uint8_t cmd_data;
	vTaskDelay(100 / portTICK_RATE_MS);
	ESP_LOGI(TAG,"\vSetting up sensor %d on master %d",device,MASTER_NUMBER);

	/* ODR config and DLPF*/
	cmd_data = 0X00;    // DEVICE CONFIG:  accel bw =  260 Hz delay =0; gyro bw=256 delay 0.98ms -> Fs = 8kHz.
	ESP_ERROR_CHECK(i2c_master_write_slave(MASTER_NUMBER, device,CONFIG_device, &cmd_data, 1));
	cmd_data = 0x18;    // ACCEL CONFIG: 16 g
	ESP_ERROR_CHECK(i2c_master_write_slave(MASTER_NUMBER, device,ACCEL_CONFIG, &cmd_data, 1));
	cmd_data = 0x18;  	// GYRO CONFIG: 2000 º/S
	ESP_ERROR_CHECK(i2c_master_write_slave(MASTER_NUMBER, device,GYRO_CONFIG, &cmd_data, 1));
	cmd_data = 0x0;    // SMPLRT_DIV : Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
	ESP_ERROR_CHECK(i2c_master_write_slave(MASTER_NUMBER, device,SMPLRT_DIV, &cmd_data, 1));

	/* Power Mode */
	cmd_data = 0x02;    // POWER 1
	ESP_ERROR_CHECK(i2c_master_write_slave(MASTER_NUMBER, device,PWR_MGMT_1, &cmd_data, 1));
	return ESP_OK;
}

esp_err_t i2c_imu_setup_reference(i2c_port_t MASTER_NUMBER,uint8_t device){
	uint8_t cmd_data;
	vTaskDelay(100 / portTICK_RATE_MS);
	ESP_LOGI(TAG,"\vSetting up sensor %d on master %d",device,MASTER_NUMBER);

	/* ODR config and DLPF*/
	cmd_data = 0X00;    // DEVICE CONFIG:  accel bw =  260 Hz delay =0; gyro bw=256 delay 0.98ms -> Fs = 8kHz.
	ESP_ERROR_CHECK(i2c_master_write_slave(MASTER_NUMBER, device,CONFIG_device, &cmd_data, 1));
	cmd_data = 0x18;    // ACCEL CONFIG: 16 g
	ESP_ERROR_CHECK(i2c_master_write_slave(MASTER_NUMBER, device,ACCEL_CONFIG, &cmd_data, 1));
	cmd_data = 0x18;  	// GYRO CONFIG: 2000 º/S
	ESP_ERROR_CHECK(i2c_master_write_slave(MASTER_NUMBER, device,GYRO_CONFIG, &cmd_data, 1));
	cmd_data = 0x0;    // SMPLRT_DIV : Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
	ESP_ERROR_CHECK(i2c_master_write_slave(MASTER_NUMBER, device,SMPLRT_DIV, &cmd_data, 1));

	/* Power Mode */
	cmd_data = 0x00;    // POWER 1
	ESP_ERROR_CHECK(i2c_master_write_slave(MASTER_NUMBER, device,PWR_MGMT_1, &cmd_data, 1));


	return ESP_OK;
}
/* Initialize master hardware and config. for comunnication*/
esp_err_t i2c_master_init(i2c_port_t MASTER_NUMBER, int sda, int scl)
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

