//TODO: Integrate nmea parser, simplify topology

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nmea_parser.h"
#include "include/i2c_cmd.h"
#include "include/ht16k33.h"

extern uint16_t displayBuffer[8];

static const char *TAG = "gps_demo";

#define TIME_ZONE (-5)   //EDT
#define YEAR_BASE (2000) //date in GPS starts from 2000

/**
 * @brief GPS Event Handler
 *
 * @param event_handler_arg handler specific arguments
 * @param event_base event base, here is fixed to ESP_NMEA_EVENT
 * @param event_id event id
 * @param event_data event specific arguments
 */
static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    gps_t *gps = NULL;
    switch (event_id) {
    case GPS_UPDATE:
        gps = (gps_t *)event_data;
        uint16_t formatHr = gps->tim.hour + TIME_ZONE + ((gps->tim.hour + TIME_ZONE >= 0) ? 0 : 24); 
        /* print information parsed from GPS statements */
        ESP_LOGI(TAG, "%d/%d/%d %d:%d:%d => \r\n"
                 "\t\t\t\t\t\tlatitude   = %.05f°N\r\n"
                 "\t\t\t\t\t\tlongitude = %.05f°E\r\n"
                 "\t\t\t\t\t\taltitude   = %.02fm\r\n"
                 "\t\t\t\t\t\tspeed      = %fm/s",
                 gps->date.year + YEAR_BASE, gps->date.month, gps->date.day,
                 formatHr, gps->tim.minute, gps->tim.second,
                 gps->latitude, gps->longitude, gps->altitude, gps->speed);
        printf("%d", (formatHr * 100 + gps->tim.minute));
        writeNumDec(displayBuffer, (formatHr * 100 + gps->tim.minute));
        writeBuffer(I2C_ADDR_7SEG, displayBuffer);
        break;
    case GPS_UNKNOWN:
        /* print unknown statements */
        ESP_LOGW(TAG, "Unknown statement:%s", (char *)event_data);
        break;
    default:
        break;
    }
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

    /* NMEA parser configuration */
    nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
    /* init NMEA parser library */
    nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);
    /* register event handler for NMEA parser library */
    nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL);

    //vTaskDelay(50000 / portTICK_PERIOD_MS);

    /* unregister event handler */
    //nmea_parser_remove_handler(nmea_hdl, gps_event_handler);
    /* deinit NMEA parser library */
    //nmea_parser_deinit(nmea_hdl);
}