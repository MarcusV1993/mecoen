/*
 * energy_meter_storage.cpp
 *
 *  Created on: 13 de mar. de 2022
 *      Author: marcus
 */


#include "mecoen_storage.h"
#include "nvs_flash.h"

// NVS related functions
esp_err_t
init_nvs()
{
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	  ESP_ERROR_CHECK(nvs_flash_erase());
	  ret = nvs_flash_init();
	}

	return ret;
}
// NVS related functions

