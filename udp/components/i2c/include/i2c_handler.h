#include "driver/i2c.h"

#ifndef I2C_HANDLER_H
#define I2C_HANDLER_H

/* Configuration of I2C*/
#define I2C_MASTER_FREQ_HZ 100000        /* I2C master clock frequency */
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

#define NUMBER_CHANNELS 1
#define MASTER_0 0
#define MASTER_1 1

/* Master rad on I2C bus*/
esp_err_t i2c_master_read_slave(i2c_port_t i2c_num,uint8_t device ,uint8_t reg_address, uint8_t *data, size_t data_len);

/*Master write on I2C bus*/
esp_err_t i2c_master_write_slave(i2c_port_t i2c_num ,uint8_t device ,uint8_t reg_address, uint8_t *data, size_t data_len);

/* Set of registers to set after IMU has started up*/
esp_err_t i2c_imu_setup(i2c_port_t MASTER_NUMBER,uint8_t device);

/* Initialize master hardware and config. for comunnication*/
esp_err_t i2c_master_init(i2c_port_t MASTER_NUMBER, int sda, int scl);

/*initialization of the IMU sensors on each channel*/
void initialization();

#endif