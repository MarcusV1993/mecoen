/*
 * energy_meter_i2c.cpp
 *
 *  Created on: 13 de mar. de 2022
 *      Author: marcus
 *
 *  Reference: https://github.com/UncleRus/esp-idf-lib/tree/master/examples/ds3231
 */

#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ds3231.h"

#include "mecoen_time.h"
#include "mecoen_i2c.h"


i2c_dev_t dev;


void
init_rtc_ds3231(struct tm *current_time)
{
    ESP_ERROR_CHECK(i2cdev_init());

	memset(&dev, 0, sizeof(i2c_dev_t));

	ESP_ERROR_CHECK(ds3231_init_desc(&dev, 0, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));

    ESP_ERROR_CHECK(ds3231_set_time(&dev, current_time));
}

void
rtc_ds3231 (void *arg)
{
	float temp;
	struct tm time;

	while(1)
	{
// Check https://www.freertos.org/vtaskdelayuntil.html
// Try for synchronize ESP32 with RTC
// To-do: Triple sync ESP32, TRC, NTP
        vTaskDelay(pdMS_TO_TICKS(7000)); // 7 s

        if (ds3231_get_temp_float(&dev, &temp) != ESP_OK)
        {
            printf("Could not get temperature\n");
            continue;
        }

        if (ds3231_get_time(&dev, &time) != ESP_OK)
        {
            printf("Could not get time\n");
            continue;
        }

        /* float is used in printf(). you need non-default configuration in
         * sdkconfig for ESP8266, which is enabled by default for this
         * example. see sdkconfig.defaults.esp8266
         */
        printf("%04d-%02d-%02d %02d:%02d:%02d, %.2f deg Cel\n", time.tm_year + 1900 /*Add 1900 for better readability*/, time.tm_mon + 1,
            time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec, temp);
	}
}
