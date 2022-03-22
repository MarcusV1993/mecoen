/*
 * energy_meter_storage.cpp
 *
 *  Created on: 13 de mar. de 2022
 *      Author: marcus
 */


#include "nvs_flash.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_system.h"

#include "mecoen_storage.h"

static const char *TAG = "mecoen_storage";

// Handle of the wear levelling library instance
static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

// Mount path for the partition
const char *base_path = "/mecoen";


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
// end NVS related functions

// WL + FAT related functions
//void init_wl_fat_vfs()
//{
//	ESP_LOGI(TAG, "Mounting FAT filesystem");
//	// To mount device we need name of device partition, define base_path
//	// and allow format partition in case if it is new one and was not formatted before
//	const esp_vfs_fat_mount_config_t mount_config = {};
//	memset(&mount, 0, sizeof(wifi_config_t));
//			.max_files = 4,
//			.format_if_mount_failed = true,
//			.allocation_unit_size = CONFIG_WL_SECTOR_SIZE
//	};
//
//	esp_err_t err = esp_vfs_fat_spiflash_mount(base_path, "storage", &mount_config, &s_wl_handle);
//	if (err != ESP_OK) {
//		ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
//		return;
//	}
//}

// NVS related functions

