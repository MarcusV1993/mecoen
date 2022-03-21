/*
 * mecoen_wifi.cpp
 *
 *  Created on: 20 de mar. de 2022
 *      Author: marcus
 *
*   Adapted from:
* 	 https://github.com/espressif/esp-idf/tree/master/examples/wifi/getting_started/softAP
* 	 https://github.com/espressif/esp-idf/tree/master/examples/wifi/getting_started/station
 */

#include <string.h>
#include <stdlib.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "mecoen_wifi.h"
#include "mecoen_storage.h"

#include "esp_err.h"

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

static const char *TAG_WIFI = "wifi ap + station";

static int s_retry_num = 0;


// Event handler adapted from merging from examples softAP and station
static void
event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
	if (event_base == WIFI_EVENT)
	{
		switch (event_id)
		{
			case WIFI_EVENT_STA_START: // STA
printf("WIFI_EVENT_STA_START\n");
				esp_wifi_connect();
				break;

			case WIFI_EVENT_STA_DISCONNECTED: // STA
printf("WIFI_EVENT_STA_DISCONNECTED\n");
				if (s_retry_num < MECOEN_WIFI_MAXIMUM_RETRY) {
					esp_wifi_connect();
					s_retry_num++;
					ESP_LOGI(TAG_WIFI, "retry to connect to the AP");
				} else {
					xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
				}
				ESP_LOGI(TAG_WIFI,"connect to the AP fail");
				break;

			case WIFI_EVENT_AP_STACONNECTED: // AP
printf("WIFI_EVENT_AP_STACONNECTED\n");
				ESP_LOGI(TAG_WIFI, "station " MACSTR " join, AID=%d",
						 MAC2STR(((wifi_event_ap_staconnected_t*) event_data)->mac), ((wifi_event_ap_staconnected_t*) event_data)->aid);
				break;

			case WIFI_EVENT_AP_STADISCONNECTED: // AP
printf("WIFI_EVENT_AP_STADISCONNECTED\n");
				ESP_LOGI(TAG_WIFI, "station " MACSTR " leave, AID=%d",
						 MAC2STR(((wifi_event_ap_stadisconnected_t*) event_data)->mac), ((wifi_event_ap_stadisconnected_t*) event_data)->aid);
				break;

			default:
				break;
		}
	}
	else if (event_base == IP_EVENT)
	{
		if (event_id == IP_EVENT_STA_GOT_IP)
		{
			ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
printf("My IP: " IPSTR "\n", IP2STR(&event->ip_info.ip));
			ESP_LOGI(TAG_WIFI, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
			s_retry_num = 0;
			xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
		}
	}
}


void
wifi_init_ap_sta()
{
	ESP_ERROR_CHECK(init_nvs());
	ESP_LOGI(TAG_WIFI,"nvs initialized");
printf("nvs initialized\n");
    s_wifi_event_group = xEventGroupCreate(); // STA

    ESP_ERROR_CHECK(esp_netif_init()); // AP and STA
	ESP_ERROR_CHECK(esp_event_loop_create_default());  // AP and STA
printf("netif init and event loop create default\n");
    esp_netif_create_default_wifi_ap(); // AP
    esp_netif_create_default_wifi_sta(); //STA
printf("create default ap and sta");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();  // AP and STA
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));  // AP and STA
printf("inti config default\n");
	esp_event_handler_instance_t instance_any_id; // STA
	esp_event_handler_instance_t instance_got_ip; // STA

	// AP + STA merge
	ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
														ESP_EVENT_ANY_ID,
														&event_handler,
														NULL,
														&instance_any_id));
	// STA
	ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
														IP_EVENT_STA_GOT_IP,
														&event_handler,
														NULL,
														&instance_got_ip));

	// STA config
	//  convert from C to C++
	//  https://esp32.com/viewtopic.php?t=1317
    wifi_config_t wifi_sta_config = { };
    memset(&wifi_sta_config, 0, sizeof(wifi_config_t));
    strcpy(reinterpret_cast<char*>(wifi_sta_config.sta.ssid), MECOEN_WIFI_STA_SSID);
    strcpy(reinterpret_cast<char*>(wifi_sta_config.sta.password), MECOEN_WIFI_STA_PASS);
	/* Setting a password implies station will connect to all security modes including WEP/WPA.
	 * However these modes are deprecated and not advisable to be used. Incase your Access point
	 * doesn't support WPA2, these mode can be enabled by commenting below line */
	wifi_sta_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK,
	wifi_sta_config.sta.pmf_cfg.capable = true;
	wifi_sta_config.sta.pmf_cfg.required = false;
	// end STA config
printf("STA config\n");
	// AP config
    wifi_config_t wifi_ap_config = { };
    memset(&wifi_ap_config, 0, sizeof(wifi_config_t));
    strcpy(reinterpret_cast<char*>(wifi_ap_config.ap.ssid), MECOEN_WIFI_AP_SSID);
    wifi_ap_config.ap.ssid_len = strlen(MECOEN_WIFI_AP_SSID);
    strcpy(reinterpret_cast<char*>(wifi_ap_config.ap.password), MECOEN_WIFI_AP_PASS);
    wifi_ap_config.ap.max_connection = MECOEN_WIFI_AP_MAX_STA_CONN;
    wifi_ap_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;

    if (strlen(MECOEN_WIFI_AP_PASS) == 0)
    {
        wifi_ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    // end AP config
printf("AP config\n");
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA)); // AP + STA
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_sta_config));
    esp_err_t ret;
    ret = esp_wifi_start();
    ESP_ERROR_CHECK(ret);
    printf("%s\n", esp_err_to_name(ret));
//    ESP_ERROR_CHECK(esp_wifi_start()); // AP + STA

    ESP_LOGI(TAG_WIFI, "wifi_init_sta finished.");
    ESP_LOGI(TAG_WIFI, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
                 MECOEN_WIFI_AP_SSID, MECOEN_WIFI_AP_PASS, MECOEN_WIFI_AP_CHANNEL);
printf("sta and softap finished\n");

    // STA \>
    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);
printf("Event bits\n");
    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG_WIFI, "connected to ap SSID:%s password:%s",
                 MECOEN_WIFI_STA_SSID, MECOEN_WIFI_STA_PASS);
printf("connected to ap SSID:%s password:%s\n",
        MECOEN_WIFI_STA_SSID, MECOEN_WIFI_STA_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG_WIFI, "Failed to connect to SSID:%s, password:%s",
        		MECOEN_WIFI_STA_SSID, MECOEN_WIFI_STA_PASS);
printf(TAG_WIFI, "Failed to connect to SSID:%s, password:%s\n",
		MECOEN_WIFI_STA_SSID, MECOEN_WIFI_STA_PASS);
    } else {
        ESP_LOGE(TAG_WIFI, "UNEXPECTED EVENT");
printf("UNEXPECTED EVENT\n");
    }
printf("Init end\n");
    /* The event will not be processed after unregister */
//    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
//    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
//    vEventGroupDelete(s_wifi_event_group);

}
