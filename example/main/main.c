#include "ads1115.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO 22        // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO 21        // GPIO number for I2C master data
#define I2C_MASTER_NUM I2C_NUM_0    // I2C port number
#define I2C_MASTER_FREQ_HZ 400000   // I2C master clock frequency

void app_main(void) {
    // Initialize I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    // Initialize ADS1115
    ads1115_t adc;
    ads1115_init(&adc, I2C_MASTER_NUM, ADS1115_I2C_ADDR_GND);
    ads1115_set_gain(&adc, ADS1115_GAIN_4V096);
    ads1115_set_data_rate(&adc, ADS1115_RATE_128SPS);

    // Read channels
    while (1) {
        int16_t value;
        float voltage;
        
        for (uint8_t ch = 0; ch < 4; ch++) {
            if (ads1115_read_channel(&adc, ch, &value) == ESP_OK) {
                voltage = ads1115_raw_to_voltage(&adc, value);
                printf("Channel %d: %d (%.3f V)\n", ch, value, voltage);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}