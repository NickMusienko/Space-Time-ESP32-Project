#include "include/i2c_cmd.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/i2c.h"

esp_err_t i2c_init(void)
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