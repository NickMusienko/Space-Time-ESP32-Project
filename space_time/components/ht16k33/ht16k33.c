#include "include/ht16k33.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/i2c.h"

uint16_t displayBuffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};

static const char numbertable[] = {
  0x3F, /* 0 */
  0x06, /* 1 */
  0x5B, /* 2 */
  0x4F, /* 3 */
  0x66, /* 4 */
  0x6D, /* 5 */
  0x7D, /* 6 */
  0x07, /* 7 */
  0x7F, /* 8 */
  0x6F, /* 9 */
  0x77, /* a */
  0x7C, /* b */
  0x39, /* C */
  0x5E, /* d */
  0x79, /* E */
  0x71, /* F */
};

esp_err_t setBlinkRate(uint8_t slave_addr, uint8_t b)
{
    if (b > 3) b = 3;
    return i2c_master_tx_byte(slave_addr, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (b << 1));
}

esp_err_t setBrightness(uint8_t slave_addr, uint8_t b)
{
    if (b > 15) b = 15;
    return i2c_master_tx_byte(slave_addr, HT16K33_CMD_BRIGHTNESS | b);
}

void clearBuffer(uint16_t* buffer)
{
    for (int i = 0; i < 8; i++)
    {
        buffer[i] = 0;
    }
}

/* toggles decimal points (1 turns on, 0 turns off)
   buffer = buffer to pass
   bit 0 - lower dot
   bit 1 - upper dot
   bit 2 - colon
   Ex: 0b101 = 0x5 = colon and lower dot on */
   
void toggleColon(uint16_t* buffer, uint8_t state) {
  buffer[2] = (0x02 * (state >> 2)) + (0x04 * (0x01 & state >> 1) + (0x08 * (0x01 & state)));
}

/* writes a single digit to the buffer at a given position
   buffer = self-explanatory
   pos = requested position (between 1 to 4, from left to right)
   d = data value from 0 to f (to turn the digit off, set 0x10 */

void writeDigit(uint16_t* buffer, uint8_t d, uint8_t pos) {
  uint16_t idx, val;
  if (pos > 4) return;
  idx = pos;
  if (pos <= 2) idx -= 1;
  
  if (d == 0x10) val = 0x00;
  else val = numbertable[d];
  
  buffer[idx] = val;
}

/* writes an entire 4 digit number to the buffer at once in decimal
   buffer = you need this
   d = data value from 0 to 9999 */
   
void writeNumDec(uint16_t* buffer, uint16_t d) {
  int parts[4];
  int i;
  parts[3] = d % 10;
  parts[2] = d % 100 / 10;
  parts[1] = d % 1000 / 100;
  parts[0] = d / 1000;
  
  for (i = 0; i < 4; i++) {
    writeDigit(buffer, parts[i], i+1);
  }
}

/* writes an entire 4 digit number to the buffer at once in hex
   buffer = you need this
   d = data value from 0 to FFFF */
   
void writeNumHex(uint16_t* buffer, uint16_t d) {
  int parts[4];
  int i;
  parts[3] = d % 0x10;
  parts[2] = d % 0x100 / 0x10;
  parts[1] = d % 0x1000 / 0x100;
  parts[0] = d / 0x1000;
  
  for (i = 0; i < 4; i++) {
    writeDigit(buffer, parts[i], i+1);
  }
}

/* writes an entire 4 digit number to the buffer at once from a string (no hex)
   If 00:XX is input, 12:XX will be output
   Supresses leading zero too.
   buffer = you need this
   str = input string (only looks at first four characters, so formatting should not matter) */

void writeNumASCII(uint16_t* buffer, char* str, bool time) {
 int parts[4];
 int i;
 for (i = 0; i < 4; i++) {
  parts[i] = str[i] & 0x0F;
 }
 
 if (time == true && parts[0] == 0 && parts[1] == 0) {    // 12:00am correction
   parts[0] = 1;
   parts[1] = 2;
 } else if (parts[0] == 0) {
   parts[0] = 0x10;
 }
 
 for (i = 0; i < 4; i++) {
  writeDigit(buffer, parts[i], i+1);
 }
}


esp_err_t writeBuffer(uint8_t slave_addr, uint16_t* buffer)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // create the link
    i2c_master_start(cmd); // start link
    i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN); // write slave ID, r/w bit
    i2c_master_write_byte(cmd, 0x00, I2C_ACK_CHECK_EN);
    for (int i = 0; i < 8; i++)
    {
        uint16_t out = buffer[i];
        i2c_master_write_byte(cmd, out & 0xFF, I2C_ACK_CHECK_EN);
        i2c_master_write_byte(cmd, out >> 8, I2C_ACK_CHECK_EN);
    }
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT_NUMBER, cmd, 50 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t ht16k33_init(uint8_t slave_addr)
{
    esp_err_t ret = i2c_master_tx_byte(slave_addr, 0x21);
    if (ret == ESP_OK) setBlinkRate(slave_addr, 0);
    if (ret == ESP_OK) setBrightness(slave_addr, 15);
    return ret;
}
