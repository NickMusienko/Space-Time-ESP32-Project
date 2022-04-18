#pragma once

#include "esp_system.h"
#include "include/i2c_cmd.h"

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_ADDR_7SEG				0x70
#define HT16K33_BLINK_CMD           0x80
#define HT16K33_BLINK_DISPLAYON     0x01
#define HT16K33_BLINK_OFF              0
#define HT16K33_BLINK_2HZ              1
#define HT16K33_BLINK_1HZ              2
#define HT16K33_BLINK_HALFHZ           3

#define HT16K33_CMD_BRIGHTNESS      0xE0

esp_err_t setBlinkRate(uint8_t slave_addr, uint8_t b);
esp_err_t setBrightness(uint8_t slave_addr, uint8_t b);
void clearBuffer(uint16_t* buffer);
void toggleColon(uint16_t* buffer, uint8_t state);
void writeDigit(uint16_t* buffer, uint8_t d, uint8_t pos);
void writeNumDec(uint16_t* buffer, uint16_t d);
void writeNumHex(uint16_t* buffer, uint16_t d);
void writeNumASCII(uint16_t* buffer, char* str, bool time);
esp_err_t writeBuffer(uint8_t slave_addr, uint16_t* buffer);
esp_err_t ht16k33_init(uint8_t slave_addr);

#ifdef __cplusplus
}
#endif
