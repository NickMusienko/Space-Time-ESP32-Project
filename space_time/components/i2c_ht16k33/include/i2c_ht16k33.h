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

#define I2C_ADDR_7SEG				0x70
#define HT16K33_BLINK_CMD           0x80
#define HT16K33_BLINK_DISPLAYON     0x01
#define HT16K33_BLINK_OFF              0
#define HT16K33_BLINK_2HZ              1
#define HT16K33_BLINK_1HZ              2
#define HT16K33_BLINK_HALFHZ           3

#define HT16K33_CMD_BRIGHTNESS      0xE0

esp_err_t i2c_init(void);
esp_err_t i2c_master_tx_byte(uint8_t slave_addr, uint8_t data);
esp_err_t i2c_master_tx(uint8_t slave_addr, uint8_t* data_array, size_t data_num);
esp_err_t setBlinkRate(uint8_t slave_addr, uint8_t b);
esp_err_t setBrightness(uint8_t slave_addr, uint8_t b);
void clearBuffer(uint16_t* buffer);
void toggleColon(uint16_t* buffer, uint8_t state);
void writeDigit(uint16_t* buffer, uint8_t d, uint8_t pos);
void writeNumDec(uint16_t* buffer, uint16_t d);
void writeNumHex(uint16_t* buffer, uint16_t d);
void writeNumASCII(uint16_t* buffer, char* str);
esp_err_t writeBuffer(uint8_t slave_addr, uint16_t* buffer);
esp_err_t ht16k33_init(uint8_t slave_addr);

#ifdef __cplusplus
}
#endif
