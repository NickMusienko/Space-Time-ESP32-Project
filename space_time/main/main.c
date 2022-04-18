#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/i2c.h"
#include "include/i2c_cmd.h"
#include "include/ht16k33.h"

extern uint16_t displayBuffer[8];

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
        writeNumDec(displayBuffer, 1235);
        writeBuffer(I2C_ADDR_7SEG, displayBuffer);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        writeNumHex(displayBuffer, 0xabcd);
        writeBuffer(I2C_ADDR_7SEG, displayBuffer);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        writeNumASCII(displayBuffer, "0053", false);
        writeBuffer(I2C_ADDR_7SEG, displayBuffer);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        writeNumHex(displayBuffer, 0xbeef);
        writeBuffer(I2C_ADDR_7SEG, displayBuffer);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        toggleColon(displayBuffer, 0b101);
        writeBuffer(I2C_ADDR_7SEG, displayBuffer);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        toggleColon(displayBuffer, 0b010);
        writeBuffer(I2C_ADDR_7SEG, displayBuffer);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        toggleColon(displayBuffer, 0b000);
        writeBuffer(I2C_ADDR_7SEG, displayBuffer);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        writeNumHex(displayBuffer, 0xdead);
        writeBuffer(I2C_ADDR_7SEG, displayBuffer);
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        

        //printf("Entered loop successfully!");
    }
}
