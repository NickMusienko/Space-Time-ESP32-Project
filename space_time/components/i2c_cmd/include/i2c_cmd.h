#pragma once

#include "esp_system.h"

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_PORT_NUMBER				I2C_NUM_1		//I2C port number
#define I2C_SCL_GPIO				19				//GPIO pin
#define I2C_SDA_GPIO				18				//GPIO pin
#define I2C_FREQ_HZ					100000			//!< I2C master clock frequency
#define I2C_ACK_CHECK_EN			0x1
#define I2C_ACK_CHECK_DIS			0x0
#define I2C_ACK_VAL					0x0
#define I2C_NACK_VAL				0x1

esp_err_t i2c_init(void);
esp_err_t i2c_master_tx_byte(uint8_t slave_addr, uint8_t data);
esp_err_t i2c_master_tx(uint8_t slave_addr, uint8_t* data_array, size_t data_num);


#ifdef __cplusplus
}
#endif
