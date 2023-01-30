/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "i2c-lcd.h"
#include "keypad.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#define lock 4

static const char *TAG = "i2c-simple-example";

char defaultPassword[5] = "1212";
char keyBuffer[5] = "";
const char emptyString[] = "";

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_NUM_0;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}


void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    ///                     R1  R2  R3  R4  C1  C2  C3  C4 
    gpio_num_t keypad[8] = {13, 12, 14, 27, 26, 25, 33, 32};

    /// Initialize keyboard
    keypad_initalize(keypad);

    gpio_set_direction(lock, GPIO_MODE_OUTPUT);
    gpio_set_level(lock, 0);
    
    lcd_init();
    lcd_clear();
    lcd_put_cur(0, 1);
    lcd_send_string("Enter password: ");
    lcd_put_cur(1, 3);
while(1){
        char keypressed = keypad_getkey();
        if(keypressed == '#'){
            checkPassword(keyBuffer, defaultPassword, lock);
        }

        if(keypressed == '*'){
            enterPassword(keyBuffer);
        }

        if(keypressed == '0'||keypressed == '1'||keypressed == '2'||keypressed == '3'||
        keypressed == '4'||keypressed == '5'||keypressed == '6'||keypressed == '7'||
        keypressed == '8'||keypressed == '9'){
            if(keypressed == '0') strcat(keyBuffer, "0");
            if(keypressed == '1') strcat(keyBuffer, "1");
            if(keypressed == '2') strcat(keyBuffer, "2");
            if(keypressed == '3') strcat(keyBuffer, "3");
            if(keypressed == '4') strcat(keyBuffer, "4");
            if(keypressed == '5') strcat(keyBuffer, "5");
            if(keypressed == '6') strcat(keyBuffer, "6");
            if(keypressed == '7') strcat(keyBuffer, "7");
            if(keypressed == '8') strcat(keyBuffer, "8");
            if(keypressed == '9') strcat(keyBuffer, "9");
            //lcd_send_string(keypressed);
            lcd_send_data(keypressed);
        }
                vTaskDelay(100 / portTICK_PERIOD_MS);

    }
        vTaskDelay(100 / portTICK_PERIOD_MS);

}
