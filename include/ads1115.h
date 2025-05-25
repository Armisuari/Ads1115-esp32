/**
 * @file ads1115.h
 * @brief ADS1115 16-bit ADC driver for ESP-IDF
 * @version 1.0
 * @date 2023-10-20
 * @author Your Name
 */

 #ifndef _ADS1115_H_
 #define _ADS1115_H_
 
 #include <stdint.h>
 #include <stdbool.h>
 #include "driver/i2c.h"
 #include "esp_err.h"
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 #define ADS1115_I2C_ADDR_GND 0x48  // Address pin connected to GND
 #define ADS1115_I2C_ADDR_VDD 0x49   // Address pin connected to VDD
 #define ADS1115_I2C_ADDR_SDA 0x4A   // Address pin connected to SDA
 #define ADS1115_I2C_ADDR_SCL 0x4B   // Address pin connected to SCL
 
 // Register addresses
 #define ADS1115_REG_CONVERSION 0x00
 #define ADS1115_REG_CONFIG     0x01
 #define ADS1115_REG_LO_THRESH  0x02
 #define ADS1115_REG_HI_THRESH  0x03
 
 // Configuration options
 typedef enum {
     ADS1115_MUX_AIN0_AIN1 = 0x0000, // Differential P = AIN0, N = AIN1 (default)
     ADS1115_MUX_AIN0_AIN3 = 0x1000, // Differential P = AIN0, N = AIN3
     ADS1115_MUX_AIN1_AIN3 = 0x2000, // Differential P = AIN1, N = AIN3
     ADS1115_MUX_AIN2_AIN3 = 0x3000, // Differential P = AIN2, N = AIN3
     ADS1115_MUX_AIN0_GND  = 0x4000, // Single-ended AIN0
     ADS1115_MUX_AIN1_GND  = 0x5000, // Single-ended AIN1
     ADS1115_MUX_AIN2_GND  = 0x6000, // Single-ended AIN2
     ADS1115_MUX_AIN3_GND  = 0x7000  // Single-ended AIN3
 } ads1115_mux_t;
 
 typedef enum {
     ADS1115_GAIN_6V144   = 0x0000, // ±6.144V range = Gain 2/3
     ADS1115_GAIN_4V096   = 0x0200, // ±4.096V range = Gain 1
     ADS1115_GAIN_2V048   = 0x0400, // ±2.048V range = Gain 2 (default)
     ADS1115_GAIN_1V024   = 0x0600, // ±1.024V range = Gain 4
     ADS1115_GAIN_0V512   = 0x0800, // ±0.512V range = Gain 8
     ADS1115_GAIN_0V256   = 0x0A00  // ±0.256V range = Gain 16
 } ads1115_gain_t;
 
 typedef enum {
     ADS1115_RATE_8SPS   = 0x0000, // 8 samples per second
     ADS1115_RATE_16SPS  = 0x0020, // 16 samples per second
     ADS1115_RATE_32SPS  = 0x0040, // 32 samples per second
     ADS1115_RATE_64SPS  = 0x0060, // 64 samples per second
     ADS1115_RATE_128SPS = 0x0080, // 128 samples per second (default)
     ADS1115_RATE_250SPS = 0x00A0, // 250 samples per second
     ADS1115_RATE_475SPS = 0x00C0, // 475 samples per second
     ADS1115_RATE_860SPS = 0x00E0  // 860 samples per second
 } ads1115_data_rate_t;
 
 typedef enum {
     ADS1115_MODE_CONTINUOUS = 0x0000, // Continuous conversion mode
     ADS1115_MODE_SINGLE     = 0x0100  // Single-shot mode (default)
 } ads1115_mode_t;
 
 typedef struct {
     i2c_port_t i2c_port;
     uint8_t i2c_addr;
     ads1115_gain_t gain;
     ads1115_data_rate_t data_rate;
     ads1115_mode_t mode;
 } ads1115_t;
 
 /**
  * @brief Initialize ADS1115 device
  * 
  * @param dev Pointer to ADS1115 device structure
  * @param i2c_port I2C port number
  * @param i2c_addr I2C address (use ADS1115_I2C_ADDR_* macros)
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t ads1115_init(ads1115_t *dev, i2c_port_t i2c_port, uint8_t i2c_addr);
 
 /**
  * @brief Set the gain (PGA) setting
  * 
  * @param dev Pointer to ADS1115 device structure
  * @param gain Gain setting
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t ads1115_set_gain(ads1115_t *dev, ads1115_gain_t gain);
 
 /**
  * @brief Set the data rate
  * 
  * @param dev Pointer to ADS1115 device structure
  * @param data_rate Data rate setting
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t ads1115_set_data_rate(ads1115_t *dev, ads1115_data_rate_t data_rate);
 
 /**
  * @brief Set the operation mode
  * 
  * @param dev Pointer to ADS1115 device structure
  * @param mode Operation mode (continuous or single-shot)
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t ads1115_set_mode(ads1115_t *dev, ads1115_mode_t mode);
 
 /**
  * @brief Read a single-ended channel
  * 
  * @param dev Pointer to ADS1115 device structure
  * @param channel Channel number (0-3)
  * @param value Pointer to store the read value
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t ads1115_read_channel(ads1115_t *dev, uint8_t channel, int16_t *value);
 
 /**
  * @brief Read a differential channel pair
  * 
  * @param dev Pointer to ADS1115 device structure
  * @param mux Mux setting for differential pair
  * @param value Pointer to store the read value
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t ads1115_read_differential(ads1115_t *dev, ads1115_mux_t mux, int16_t *value);
 
 /**
  * @brief Start a single-shot conversion (only needed in single-shot mode)
  * 
  * @param dev Pointer to ADS1115 device structure
  * @param mux Mux setting for the conversion
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t ads1115_start_conversion(ads1115_t *dev, ads1115_mux_t mux);
 
 /**
  * @brief Check if conversion is complete
  * 
  * @param dev Pointer to ADS1115 device structure
  * @param ready Pointer to store the ready status (true if ready)
  * @return esp_err_t ESP_OK on success, error code otherwise
  */
 esp_err_t ads1115_conversion_ready(ads1115_t *dev, bool *ready);
 
 /**
  * @brief Get the voltage corresponding to a raw ADC value
  * 
  * @param dev Pointer to ADS1115 device structure
  * @param value Raw ADC value
  * @return float Voltage in volts
  */
 float ads1115_raw_to_voltage(ads1115_t *dev, int16_t value);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif // _ADS1115_H_