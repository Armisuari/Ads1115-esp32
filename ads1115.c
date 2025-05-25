/**
 * @file ads1115.c
 * @brief ADS1115 16-bit ADC driver implementation for ESP-IDF
 * @version 1.0
 * @date 2023-10-20
 * @author Your Name
 */

#include "ads1115.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"
 
 static const char *TAG = "ADS1115";
 
 // Helper function for I2C writes
 static esp_err_t ads1115_write_register(ads1115_t *dev, uint8_t reg, uint16_t value) {
     uint8_t buf[3] = {reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
     
     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
     i2c_master_start(cmd);
     i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
     i2c_master_write(cmd, buf, sizeof(buf), true);
     i2c_master_stop(cmd);
     
     esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(1000));
     i2c_cmd_link_delete(cmd);
     
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to write register 0x%02x: %s", reg, esp_err_to_name(ret));
     }
     return ret;
 }
 
 // Helper function for I2C reads
 static esp_err_t ads1115_read_register(ads1115_t *dev, uint8_t reg, uint16_t *value) {
     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
     i2c_master_start(cmd);
     i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
     i2c_master_write_byte(cmd, reg, true);
     i2c_master_start(cmd);
     i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_READ, true);
     
     uint8_t buf[2];
     i2c_master_read(cmd, buf, sizeof(buf), I2C_MASTER_LAST_NACK);
     i2c_master_stop(cmd);
     
     esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(1000));
     i2c_cmd_link_delete(cmd);
     
     if (ret == ESP_OK) {
         *value = (buf[0] << 8) | buf[1];
     } else {
         ESP_LOGE(TAG, "Failed to read register 0x%02x: %s", reg, esp_err_to_name(ret));
     }
     return ret;
 }
 
 esp_err_t ads1115_init(ads1115_t *dev, i2c_port_t i2c_port, uint8_t i2c_addr) {
     dev->i2c_port = i2c_port;
     dev->i2c_addr = i2c_addr;
     dev->gain = ADS1115_GAIN_2V048;
     dev->data_rate = ADS1115_RATE_128SPS;
     dev->mode = ADS1115_MODE_SINGLE;
     
     // Verify communication by reading the config register
     uint16_t config;
     esp_err_t ret = ads1115_read_register(dev, ADS1115_REG_CONFIG, &config);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to communicate with ADS1115 at address 0x%02x", i2c_addr);
         return ret;
     }
     
     ESP_LOGI(TAG, "ADS1115 initialized at address 0x%02x", i2c_addr);
     return ESP_OK;
 }
 
 esp_err_t ads1115_set_gain(ads1115_t *dev, ads1115_gain_t gain) {
     dev->gain = gain;
     return ESP_OK;
 }
 
 esp_err_t ads1115_set_data_rate(ads1115_t *dev, ads1115_data_rate_t data_rate) {
     dev->data_rate = data_rate;
     return ESP_OK;
 }
 
 esp_err_t ads1115_set_mode(ads1115_t *dev, ads1115_mode_t mode) {
     dev->mode = mode;
     return ESP_OK;
 }
 
 esp_err_t ads1115_read_channel(ads1115_t *dev, uint8_t channel, int16_t *value) {
     if (channel > 3) {
         ESP_LOGE(TAG, "Invalid channel number %d", channel);
         return ESP_ERR_INVALID_ARG;
     }
     
     ads1115_mux_t mux = ADS1115_MUX_AIN0_GND + (channel << 12);
     return ads1115_read_differential(dev, mux, value);
 }
 
 esp_err_t ads1115_read_differential(ads1115_t *dev, ads1115_mux_t mux, int16_t *value) {
     esp_err_t ret;
     
     if (dev->mode == ADS1115_MODE_SINGLE) {
         // Configure and start single-shot conversion
         uint16_t config = 0x8000 | // OS: Start single conversion
                          mux | 
                          dev->gain | 
                          dev->mode | 
                          dev->data_rate | 
                          0x0003; // Comparator disabled
         
         ret = ads1115_write_register(dev, ADS1115_REG_CONFIG, config);
         if (ret != ESP_OK) {
             return ret;
         }
         
         // Wait for conversion to complete
         bool ready = false;
         uint32_t timeout_ms = 1000 / (1 << (dev->data_rate >> 5)) + 10; // Approximate conversion time + margin
         
         for (uint32_t i = 0; i < timeout_ms / 10; i++) {
             vTaskDelay(pdMS_TO_TICKS(10));
             ret = ads1115_conversion_ready(dev, &ready);
             if (ret != ESP_OK || ready) {
                 break;
             }
         }
         
         if (!ready) {
             ESP_LOGE(TAG, "Conversion timeout");
             return ESP_ERR_TIMEOUT;
         }
     }
     
     // Read the conversion result
     uint16_t raw_value;
     ret = ads1115_read_register(dev, ADS1115_REG_CONVERSION, &raw_value);
     if (ret == ESP_OK) {
         *value = (int16_t)raw_value;
     }
     
     return ret;
 }
 
 esp_err_t ads1115_start_conversion(ads1115_t *dev, ads1115_mux_t mux) {
     if (dev->mode != ADS1115_MODE_SINGLE) {
         return ESP_OK; // No action needed in continuous mode
     }
     
     uint16_t config = 0x8000 | // OS: Start single conversion
                      mux | 
                      dev->gain | 
                      dev->mode | 
                      dev->data_rate | 
                      0x0003; // Comparator disabled
     
     return ads1115_write_register(dev, ADS1115_REG_CONFIG, config);
 }
 
 esp_err_t ads1115_conversion_ready(ads1115_t *dev, bool *ready) {
     uint16_t config;
     esp_err_t ret = ads1115_read_register(dev, ADS1115_REG_CONFIG, &config);
     if (ret == ESP_OK) {
         *ready = (config & 0x8000) != 0;
     }
     return ret;
 }
 
 float ads1115_raw_to_voltage(ads1115_t *dev, int16_t value) {
     float fs_range;
     
     switch (dev->gain) {
         case ADS1115_GAIN_2V048: fs_range = 2.048f; break;
         case ADS1115_GAIN_4V096: fs_range = 4.096f; break;
         case ADS1115_GAIN_1V024: fs_range = 1.024f; break;
         case ADS1115_GAIN_0V512: fs_range = 0.512f; break;
         case ADS1115_GAIN_0V256: fs_range = 0.256f; break;
         case ADS1115_GAIN_6V144: fs_range = 6.144f; break;
         default: fs_range = 2.048f; break;
     }
     
     return (value * fs_range) / 32768.0f;
 }