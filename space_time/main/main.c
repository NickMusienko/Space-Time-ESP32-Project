#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/i2c.h"

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

uint16_t displayBuffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};



static esp_err_t i2c_init(void)
{
    int i2c_master_port = I2C_PORT_NUMBER;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_GPIO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = I2C_SCL_GPIO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;
    if (i2c_param_config(i2c_master_port, &conf) == ESP_OK) return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
    else return i2c_param_config(i2c_master_port, &conf);
}

esp_err_t i2c_master_tx_byte(uint8_t slave_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // create the link
    i2c_master_start(cmd); // start link
    i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN); // write slave ID, r/w bit
    i2c_master_write_byte(cmd, data, I2C_ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT_NUMBER, cmd, 50 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    // if (error != 0) {
    //     if (ret != ESP_OK) printf("Unable to send data.\n");
    // }
    return ret;
}

esp_err_t i2c_master_tx(uint8_t slave_addr, uint8_t* data_array, size_t data_num)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // create the link
    i2c_master_start(cmd); // start link
    i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN); // write slave ID, r/w bit
    i2c_master_write(cmd, data_array, data_num, I2C_ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT_NUMBER, cmd, 50 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    // if (error != 0) {
    //     if (ret != ESP_OK) printf("Unable to send data.\n");
    // }
    return ret;
}

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

void writeNumASCII(uint16_t* buffer, char* str) {
 int parts[4];
 int i;
 for (i = 0; i < 4; i++) {
  parts[i] = str[i] & 0x0F;
 }
 
 if (parts[0] == 0 && parts[1] == 0) {    // 12:00am correction
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

void app_main(void)
{
    if (i2c_init() != ESP_OK) {
        printf("Bus unable to be initalized\n");
    } else {
        if (ht16k33_init(I2C_ADDR_7SEG) != ESP_OK) printf("Unable to initalize HT16K33.\n");
        clearBuffer(displayBuffer);
        writeBuffer(I2C_ADDR_7SEG, displayBuffer);
    }
    
    while (true)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        // writeNumDec(displayBuffer, 1234);
        // writeBuffer(I2C_ADDR_7SEG, displayBuffer);
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        // writeNumHex(displayBuffer, 0xabcd);
        // writeBuffer(I2C_ADDR_7SEG, displayBuffer);
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        // writeNumASCII(displayBuffer, "0053");
        // writeBuffer(I2C_ADDR_7SEG, displayBuffer);
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        // writeNumHex(displayBuffer, 0xbeef);
        // writeBuffer(I2C_ADDR_7SEG, displayBuffer);
        // vTaskDelay(1000 / portTICK_PERIOD_MS);

        // toggleColon(displayBuffer, 0b101);
        // writeBuffer(I2C_ADDR_7SEG, displayBuffer);
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        // toggleColon(displayBuffer, 0b010);
        // writeBuffer(I2C_ADDR_7SEG, displayBuffer);
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        // toggleColon(displayBuffer, 0b000);
        // writeBuffer(I2C_ADDR_7SEG, displayBuffer);
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        writeNumHex(displayBuffer, 0xbeef);
        writeBuffer(I2C_ADDR_7SEG, displayBuffer);
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        writeDigit(displayBuffer, 0xA, 1);
        writeDigit(displayBuffer, 0xA, 4);
        displayBuffer[1] = 0b0110111;
        displayBuffer[3] = 0b0110111;
        writeBuffer(I2C_ADDR_7SEG, displayBuffer);
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        //printf("Entered loop successfully!");
    }
}
