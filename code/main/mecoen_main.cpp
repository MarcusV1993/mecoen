/*
 * Final project to achieve degree in Electrical Engineering with Emphasis in Electronic Systems at University of São Paulo
 * Project: Real Time Home Energy Meter with Web Application Interface
 * Author: Marcus Vinicius Gonçalves Mendes
 * Advisors: Roseli de Deus Lopes (EPUSP), Marcelo Knörich Zuffo (EPUSP), Alberto Ferreira de Sousa (UFES)
 *
*/

/*
 * References:
 ** ADC: Previous project from Prof. Alberto Ferreira
 ** Wi-Fi AP: https://github.com/espressif/esp-idf/tree/master/examples/wifi/getting_started/softAP
 ** Wi-Fi Station: https://github.com/espressif/esp-idf/tree/master/examples/wifi/getting_started/station
 ** Web server: https://github.com/caiomb/esp32-http_webserver
 ** NTP: https://github.com/espressif/esp-idf/tree/master/examples/protocols/sntp
 ** I2C (DS3231): https://github.com/UncleRus/esp-idf-lib/tree/master/examples/ds3231
 ** FFT: https://github.com/espressif/esp-dsp/blob/master/examples/fft/main/dsps_fft_main.c
*/

/*
 * code structure:
 * includes
 *   includes std c libraries
 *   includes esp-idf libraries
 *     includes esp-idf system
 *     includes esp -idf error handling
 *     includes esp-idf freeRTOS
 *     includes -esp-idf time
 *       includes esp-idf time ntp
 *     includes esp-idf ADC
 *     includes esp-idf storage
 *     includes esp-idf wi-fi
 *       includes esp-idf wi-fi library for web server
 *   includes esp-lib | rtc ds3231
 *   includes esp-dsp | fft
 *   includes project headers
 *
 *  const
 *    const ADC
 *      const ADC period
 *      const ADC ports and configuration
 *      const ADC voltage sensor
 *      const ADC current sensor
 *    const time
 *    const wi-fi
 *    const storage
 *    const web server
 *
 *  global variables
 *    global variables semaphores
 *    global variables time
 *      global variables time ntp
 *    global variables ADC
 *    global variables wi-fi
 *    global variables storage
 *    global variables i2c
 *    global variables FFT
 *      global variables FFT Window coefficients
 *    global variables web server
 *
 *  functions
 *    functions variable initializer
 *    functions time
 *      functions time ntp
 *    functions ADC
 *    functions storage
 *    functions storage NVS
 *    functions wi-fi
 *    functions web server
 *    functions i2c | ds3231
 *    functions FFT
 *
 *  main
*/


// includes
//// includes std c libraries
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include <sys/time.h>
#include <time.h>
//// end includes std c libraries


//// includes esp-idf libraries
////// includes esp-idf system
#include "esp_attr.h" // memmory types
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
////// end includes esp-idf system


////// includes esp -idf error handling
#include "esp_err.h"
////// end includes esp -idf error handling


////// includes esp-idf freeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
////// end includes esp-idf freeRTOS


////// includes -esp-idf time
#include "esp_timer.h"
#include "driver/gptimer.h"

//////// includes esp-idf time ntp
#include "esp_sntp.h"
//////// end includes esp-idf time ntp
////// end includes -esp-idf time


////// includes esp-idf ADC
#include "esp_adc_cal.h"
#include "driver/adc.h"
#include "driver/gpio.h"
////// end includes esp-idf ADC


////// includes esp-idf storage
#include "nvs_flash.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
////// end includes esp-idf storage


////// includes esp-idf wi-fi
#include "esp_wifi.h"

#include "lwip/err.h"
#include "lwip/sys.h"

//////// includes esp-idf wi-fi library for web server
#include "lwip/api.h"
//////// includes esp-idf wi-fi library for web server
////// end includes esp-idf wi-fi
//// end includes esp-idf libraries


//// includes esp-lib
#include "ds3231.h"
//// end includes esp-lib


//// includes esp-dsp
#include "esp_dsp.h" // https://github.com/espressif/esp-dsp
//// end includes esp-dsp

//// includes project headers
#include "mecoen_definitions.h"
//// end includes project headers
// end includes


// const
//// const ADC
////// const ADC period
static constexpr int SAMPLING_PERIOD_US = (1e6 / SAMPLING_FREQUENCY); // Real sampling frequency slightly lower than 1e6/SAMPLING_PERIOD_US
static constexpr int ticks_1s = pdMS_TO_TICKS(1000);
////// end const ADC period

////// const ADC ports and configuration
static constexpr adc_channel_t channel_v = ADC_CHANNEL_6;     // GPIO34
static constexpr adc_channel_t channel_i = ADC_CHANNEL_7;	  // GPIO35
static constexpr adc_atten_t atten = ADC_ATTEN_DB_11; // max = 3.9V -> https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/adc.html
static constexpr adc_unit_t unit = ADC_UNIT_1;
////// end const ADC ports and configuration
//// end const ADC


//// const time
static const char *TAG_TIME = "mecoen_time";
//// end const time


//// const wi-fi
static const char *TAG_WIFI = "wifi ap + station";
//// end const wi-fi


//// const storage
static constexpr int storage_period_s = STORAGE_PERIOD * 60;
#if _MECOEN_STORAGE_
static const char *base_path = "/mecoen"; // Mount path for the partition
#endif
//// end const storage


//// const web server
constexpr static char http_html_hdr[] = "HTTP/1.1 200 OK\r\nContent-type: text/html\r\n\r\n";
//// end const web server
// end const


// global variables
static Circuit_phase phase_a;
constexpr int n_array_copy_length = N_ARRAY_LENGTH / REASON;
static float phase_copy[n_array_copy_length][3];


//// global variables semaphores task handles
static TaskHandle_t task_adc = NULL;
#if _MECOEN_FFT_
static TaskHandle_t  task_fft = NULL;
#endif

static SemaphoreHandle_t semaphore_adc_main;
static SemaphoreHandle_t semaphore_adc;
//// end global variables semaphores


//// global variables time
////// global variables time ntp
/* Variable holding number of times ESP32 restarted since first boot.
 * It is placed into RTC memory using RTC_DATA_ATTR and
 * maintains its value when ESP32 wakes from deep sleep.
 */
RTC_DATA_ATTR static int boot_count = 0;
////// end global variables time ntp
//// end global variables time


//// global variables ADC
static esp_adc_cal_characteristics_t *adc_chars;
static float zmpt101b_vdc;
static float  sct013_vdc;
//// end global variables ADC


//// global variables wi-fi
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
//// end global variables wi-fi


//// global variables storage
#if _MECOEN_STORAGE_
static wl_handle_t s_wl_handle = WL_INVALID_HANDLE; // Handle of the wear levelling library instance
#endif
//// end global variables storage


//// global variables i2c
static i2c_dev_t dev;
//// end global variables i2c


//// global variables FFT
static float effective_sample_period_us = SAMPLING_PERIOD_US;
////// global variables FFT Window coefficients
__attribute__((aligned(16)))
float wind[N_ARRAY_LENGTH];
////// global variables FFT Window coefficients
//// end global variables FFT


//// global variables web server
static char http_index_html_server[] = "\
<html>\
<body>\
<center><h1>Projeto de Graduacao em Engenharia Eletrica com Enfase em Eletronica e Sistemas</h1></center>\
<center><h1>Medidor de Consumo de Energia Eletrica Domestico de Tempo Real com Interface Via Aplicativo Web</h1></center>\
<center>Count #                                                                                                 </center>\
<center>#                                                                                                       </center>\
<center>#                                                                                                       </center>\
</body>\
</html>";

static char message[100];
//// end global variables web server
// end global variables


// functions
//// functions time
unsigned long IRAM_ATTR micros()
{
    return (unsigned long) (esp_timer_get_time());
}


void IRAM_ATTR delayMicroseconds(uint32_t us)
{
    uint32_t m = micros();
    if(us){
        uint32_t e = (m + us);
        if(m > e){ //overflow
            while(micros() > e){
                NOP();
            }
        }
        while(micros() < e){
            NOP();
        }
    }
}


////// functions time ntp
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_CUSTOM
void sntp_sync_time(struct timeval *tv)
{
   settimeofday(tv, NULL);
   ESP_LOGI(TAG_TIME, "Time is synchronized from custom code");
   sntp_set_sync_status(SNTP_SYNC_STATUS_COMPLETED);
}
#endif


void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG_TIME, "Notification of a time synchronization event");
}


static void initialize_sntp(void)
{
    ESP_LOGI(TAG_TIME, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);

/*
 * If 'NTP over DHCP' is enabled, we set dynamic pool address
 * as a 'secondary' server. It will act as a fallback server in case that address
 * provided via NTP over DHCP is not accessible
 */
#if LWIP_DHCP_GET_NTP_SRV && SNTP_MAX_SERVERS > 1
    sntp_setservername(1, "pool.ntp.org");

#if LWIP_IPV6 && SNTP_MAX_SERVERS > 2          // statically assigned IPv6 address is also possible
    ip_addr_t ip6;
    if (ipaddr_aton("2a01:3f7::1", &ip6)) {    // ipv6 ntp source "ntp.netnod.se"
        sntp_setserver(2, &ip6);
    }
#endif  /* LWIP_IPV6 */

#else   /* LWIP_DHCP_GET_NTP_SRV && (SNTP_MAX_SERVERS > 1) */
    // otherwise, use DNS address from a pool
    sntp_setservername(0, MECOEN_SNTP_SERVER_NAME);
#endif

    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
#endif
    sntp_init();

    ESP_LOGI(TAG_TIME, "List of configured NTP servers:");

    for (uint8_t i = 0; i < SNTP_MAX_SERVERS; ++i){
        if (sntp_getservername(i)){
            ESP_LOGI(TAG_TIME, "server %d: %s", i, sntp_getservername(i));
        } else {
            // we have either IPv4 or IPv6 address, let's print it
            char buff[INET6_ADDRSTRLEN];
            ip_addr_t const *ip = sntp_getserver(i);
            if (ipaddr_ntoa_r(ip, buff, INET6_ADDRSTRLEN) != NULL)
                ESP_LOGI(TAG_TIME, "server %d: %s", i, buff);
        }
    }
}


static void obtain_time(void)
{
    initialize_sntp();

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { };
    int retry = 0;
    const int retry_count = 15;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG_TIME, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    time(&now);
	setenv("TZ", "EBST3", 1);
	tzset();
	localtime_r(&now, &timeinfo);
    timeval now_timeval = {now, 0};
    settimeofday(&now_timeval, NULL);
//char strftime_buf[64];
//strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
//printf("The current date/time is: %s\n", strftime_buf);
}


void
init_time()
{
	++boot_count;
	ESP_LOGI(TAG_TIME, "Boot count: %d", boot_count);

	time_t now;
	struct tm timeinfo;
	time(&now);
	localtime_r(&now, &timeinfo);
	setenv("TZ", "EBST3", 1);
	tzset();
	localtime_r(&now, &timeinfo);
//char strftime_buf[64];
//strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
//printf("The current date/time is: %s\n", strftime_buf);
	// Is time set? If not, tm_year will be (1970 - 1900).
	if (timeinfo.tm_year < (2016 - 1900)) {
		ESP_LOGI(TAG_TIME, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
		obtain_time();
		// update 'now' variable with current time
		time(&now);
	}

	// Set timezone to Brasilia
	setenv("TZ", "EBST3", 1);
	tzset();
	localtime_r(&now, &timeinfo);

	if (sntp_get_sync_mode() == SNTP_SYNC_MODE_SMOOTH) {
		struct timeval outdelta;
		while (sntp_get_sync_status() == SNTP_SYNC_STATUS_IN_PROGRESS) {
			adjtime(NULL, &outdelta);
			ESP_LOGI(TAG_TIME, "Waiting for adjusting time ... outdelta = %li sec: %li ms: %li us",
						(long)outdelta.tv_sec,
						outdelta.tv_usec/1000,
						outdelta.tv_usec%1000);
			vTaskDelay(2000 / portTICK_PERIOD_MS);
		}
	}
}
////// end functions time ntp
//// end functions time


//// functions ADC
static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}


static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}


void
init_adc()
{
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

	// Init breaks pontentiometer sensor (https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/adc.html)
    // Usamos a ADC_UNIT_1

	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten((adc1_channel_t) channel_v, atten);
	adc1_config_channel_atten((adc1_channel_t) channel_i, atten); // Second ADC channel

    //Characterize ADC
    adc_chars = (esp_adc_cal_characteristics_t *) calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
    printf("\nadc initialized!\n");
}


void
read_phase(void *arg)
{
	printf("\nread_phase initiated!\n");
	Circuit_phase *phase = (Circuit_phase *) arg;
	float zmpt101b_vdc_local = 0.0; // To store locally the average value of the signal
	float sct013_vdc_local = 0.0; // To store locally the average value of the signal
	int sample_num = 0;
	unsigned long start = 0;
	float sample_period = 0.0;
	TickType_t xLastWakeTime = xTaskGetTickCount();

	ulTaskNotifyTakeIndexed(INDEX_TO_WATCH, pdTRUE, portMAX_DELAY);


	while(1)
	{
		vTaskDelayUntil( &xLastWakeTime, ticks_1s );
		sample_num = 0;

		sample_period = 0.0;
		start = micros();

		while(sample_num < N_ARRAY_LENGTH)
		{
			sample_period += (float)(micros() - start);
			start = micros();
			// Start sampling
			phase_a.voltage.samples[sample_num] = esp_adc_cal_raw_to_voltage(adc1_get_raw((adc1_channel_t) channel_v), adc_chars);
			phase_a.current.samples[sample_num] = esp_adc_cal_raw_to_voltage(adc1_get_raw((adc1_channel_t) channel_i), adc_chars);

			for(int i = 0; i < NO_OF_SAMPLES; i++)
			{
				phase_a.voltage.samples[sample_num] += esp_adc_cal_raw_to_voltage(adc1_get_raw((adc1_channel_t) channel_v), adc_chars);
				phase_a.current.samples[sample_num] += esp_adc_cal_raw_to_voltage(adc1_get_raw((adc1_channel_t) channel_i), adc_chars);
			}

			sample_num++;

			delayMicroseconds(SAMPLING_PERIOD_US);
		}

		zmpt101b_vdc_local = 0.0;
		sct013_vdc_local = 0.0;
		for(int i = 0; i < N_ARRAY_LENGTH; i++)
		{
			phase->voltage.samples[i] /= NO_OF_SAMPLES; // In mV
			phase->current.samples[i] /= NO_OF_SAMPLES; // In mV

			zmpt101b_vdc_local += phase->voltage.samples[i];
			sct013_vdc_local += phase->current.samples[i];
		}

		effective_sample_period_us = sample_period / (N_ARRAY_LENGTH - 1);
//		printf("\nsampling period: %10.0f\n", sample_period / (N_ARRAY_LENGTH - 1));

		// Copy new average to dc value
		zmpt101b_vdc = zmpt101b_vdc_local / N_ARRAY_LENGTH; // In mV
		sct013_vdc   = sct013_vdc_local / N_ARRAY_LENGTH; // In mV


		// Semaphore to main so it can copy the values
		xSemaphoreGive(semaphore_adc_main);

		// Semaphore from main that values have been copied
		xSemaphoreTake(semaphore_adc, 10 * ticks_1s);

		// Task Handle to allow FFT function to sync access to common resource
		xTaskNotifyGiveIndexed(task_fft, INDEX_TO_WATCH);

		// Task Handle to await the conclusion
		ulTaskNotifyTakeIndexed(INDEX_TO_WATCH, pdTRUE, 10 * ticks_1s);
	}
}
//// end functions ADC


//// functions storage
////// functions storage NVS
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
////// end functions storage NVS


////// functions storage wear leveling + fat
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
////// end functions storage wear leveling + fat
//// end functions storage


//// functions wi-fi
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
printf("create default ap and sta\n");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();  // AP and STA
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));  // AP and STA
printf("init config default\n");
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
//// end functions wi-fi


//// functions web server
static void
http_server_netconn_serve(struct netconn *conn)
{
    struct netbuf *inbuf;
    err_t err;
    err = netconn_recv(conn, &inbuf);
    if (err == ERR_OK)
    {
        char *output = strstr(http_index_html_server, "#");
        static int count = 0;
        sprintf(message, "%d", count++);
		strcpy(output + 1, message);
		output[strlen(message) + 1] = ';';

		output = strstr(output + 1, "#");
        sprintf(message, "Vrms = %07.2f", phase_a.voltage.rms_previous);
        strcpy(output + 1, message);
        output[strlen(message) + 1] = ';';

        output = strstr(output + 1, "#");
        sprintf(message, "Irms = %07.2f", phase_a.current.rms_previous);
		strcpy(output + 1, message);
		output[strlen(message) + 1] = '.';

        netconn_write(conn, http_html_hdr, sizeof(http_html_hdr)-1, NETCONN_NOCOPY);
        netconn_write(conn, http_index_html_server, sizeof(http_index_html_server)-1, NETCONN_NOCOPY);
    }
    netconn_close(conn);
    netbuf_delete(inbuf);
}


void
http_server(void *pvkeys)
{
    struct netconn *conn, *newconn;
    err_t err;
    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, NULL, 80);
    netconn_listen(conn);

    do {
        err = netconn_accept(conn, &newconn);
        if (err == ERR_OK)
        {
            http_server_netconn_serve(newconn);
            netconn_delete(newconn);
        }
    } while(err == ERR_OK);

    netconn_close(conn);
    netconn_delete(conn);
}
//// end functions web server


//// functions i2c
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
//// end functions i2c


//// functions FFT
esp_err_t
init_fft()
{
	esp_err_t ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);

	if (ret  != ESP_OK)
		ESP_LOGE("FFT", "Not possible to initialize FFT. Error = %i", ret);
	else
		dsps_wind_hann_f32(wind, N_ARRAY_LENGTH); // Generate hann window

    return ret;
}


void
fft_function(Signal *signal)
{
	// FFT
    dsps_fft2r_fc32(signal->y_cf, N_ARRAY_LENGTH);
    // Bit reverse
	dsps_bit_rev_fc32(signal->y_cf, N_ARRAY_LENGTH);
	// Convert one complex vector to two complex vectors
	dsps_cplx2reC_fc32(signal->y_cf, N_ARRAY_LENGTH);
}


void
fft_continuous(void *arg)
{
	printf("\nread_fft task initialized!\n");
	Circuit_phase *phase = (Circuit_phase *) arg;
	int i;
	int bin_max;
	float sampling_frequency = 0.0;
	float signal_frequency_correction = 0.0;
	float v_rms = 0.0;
	float i_rms = 0.0;
	float phase_diff = 0.0;

//	char buff[128];

	ulTaskNotifyTakeIndexed(INDEX_TO_WATCH, pdTRUE, portMAX_DELAY); // Wait for adc task to fill array

	while(1)
	{
		ulTaskNotifyTakeIndexed(INDEX_TO_WATCH, pdTRUE, portMAX_DELAY); // Wait for adc task to fill array

		sampling_frequency = 1e6 / effective_sample_period_us;
//		printf("\n");
//		for (i = 0; i < N_ARRAY_LENGTH; i++) {
//			sprintf(buff, "%12.4f %12.4f\n", phase->voltage.samples[i], phase->current.samples[i]);
//			printf("%s", buff);
//
//		}

		for (i = 0; i < N_ARRAY_LENGTH; i++)
		{
			phase->voltage.y_cf[i * 2] = phase->voltage.samples[i] * wind[i];
			phase->voltage.y_cf[(i * 2) + 1] = 0.0; // Real signal has imaginary part 0

			phase->current.y_cf[i * 2] = phase->current.samples[i] * wind[i];
			phase->current.y_cf[(i * 2) + 1] = 0.0; // Real signal has imaginary part 0
		}


		fft_function(&(phase->voltage));
		fft_function(&(phase->current));


		// Calculate magnitude and phase of the signal saving it to same array
		// Magnitude from 0 to (N_ARRAY_LENGTH / 2) and phase from N_ARRAY_LENGTH to (N_ARRAY_LENGTH + (N_ARRAY_LENGTH / 2))
		// Low Cut Filter to remove DC
		for (i = 0; i < MECOEN_FFT_LOW_CUT_BINS; i++)
		{
			// DC filter and store magnitude
			phase->voltage.y_cf[i * 2] = 0.0;
			phase->voltage.y_cf[(i * 2) + 1] = 0.0;
			// Store phase
			phase->voltage.y_cf[N_ARRAY_LENGTH + i] = 0.0;

			// DC filter and store magnitude
			phase->current.y_cf[i * 2] = 0.0;
			phase->current.y_cf[(i * 2) + 1] = 0.0;
			// Store phase
			phase->current.y_cf[N_ARRAY_LENGTH + i] = 0.0;
		}
#if 1 // Calculate magnitude and phase
		bin_max = i;
		v_rms = 0.0;
		i_rms = 0.0;
		for ( ; i < N_ARRAY_LENGTH / 2; i++)
		{
			// Magnitude
			phase->voltage.y_cf[i] = (phase->voltage.y_cf[i * 2] * phase->voltage.y_cf[i * 2]) + (phase->voltage.y_cf[i * 2 + 1] * phase->voltage.y_cf[i * 2 + 1]);
			v_rms += phase->voltage.y_cf[i];
			// In linear circuits, frequency is constant between all signals, voltage and current
			if (phase->voltage.y_cf[i] > phase->voltage.y_cf[bin_max]) {
				bin_max = i;
			}
			// Phase
			phase->voltage.y_cf[N_ARRAY_LENGTH + i] = atan2(phase->voltage.y_cf[i * 2 + 1], phase->voltage.y_cf[i * 2]);

			// Magnitude
			phase->current.y_cf[i] = (phase->current.y_cf[i * 2] * phase->current.y_cf[i * 2]) + (phase->current.y_cf[i * 2 + 1] * phase->current.y_cf[i * 2 + 1]);
			i_rms += phase->current.y_cf[i];
			// Phase
			phase->current.y_cf[N_ARRAY_LENGTH + i] = atan2(phase->current.y_cf[i * 2 + 1], phase->current.y_cf[i * 2]);
		}
		v_rms = sqrt(2 * v_rms) / (N_ARRAY_LENGTH * 1e3);
		i_rms = sqrt(2 * i_rms) / (N_ARRAY_LENGTH * 1e3);


		v_rms = v_rms < EPSILON_V_RMS ? 0.0 : v_rms * RMS_2_REAL_V;
		i_rms = i_rms < EPSILON_I_RMS ? 0.0 : i_rms * RMS_2_REAL_I;
		printf("v_rms = %5.1f i_rms = %5.1f\n", v_rms, i_rms);

		phase_diff = (v_rms < EPSILON || i_rms < EPSILON) ? 0.0 : phase->voltage.y_cf[N_ARRAY_LENGTH + i] - phase->current.y_cf[N_ARRAY_LENGTH + bin_max];
		while (phase_diff < -M_PI)
		{
			phase_diff += 2 * M_PI;
		}
		while (phase_diff > M_PI)
		{
			phase_diff -= M_PI;
		}
		phase->power.power_factor = cos(phase_diff);

		phase->power.power_factor_type = phase_diff < -EPSILON ? 'C' : (phase_diff > EPSILON ? 'I' : 'R');
		if(phase_diff < -EPSILON) {
			phase->power.power_factor_type = 'C'; // Capacitive
		} else if (phase_diff > EPSILON) {
			phase->power.power_factor_type = 'I'; // Inductive
		} else {
			phase->power.power_factor_type = 'R'; // Resistive
		}
		printf("phase_diff: %f\n", phase_diff);
//		phase->power.apparent = v_rms * i_rms;
//		phase->power.active = phase->power.apparent * cos(phase_diff);
//		phase->power.reactive = phase->power.apparent * sin(phase_diff);


		// end magnitude and phase calculation
#endif
#if 1
		if (bin_max > 0 && bin_max < (N_ARRAY_LENGTH / 2))
		{
			//// Reference: http://www.add.ece.ufl.edu/4511/references/ImprovingFFTResoltuion.pdf
			// Parabolic Interpolation
			signal_frequency_correction = (phase->voltage.y_cf[bin_max + 1] - phase->voltage.y_cf[bin_max - 1]) / \
					(2 * ((2 * phase->voltage.y_cf[bin_max]) - phase->voltage.y_cf[bin_max + 1] - phase->voltage.y_cf[bin_max - 1]));
		}
		else
		{
			signal_frequency_correction = 0.0;
		}
//		printf("\nsampling frequency: %f\n", sampling_frequency);
//		printf("\ndelta_m: %f\n", signal_frequency_correction);
		phase->power.frequency = (sampling_frequency / N_ARRAY_LENGTH) * (bin_max + signal_frequency_correction);
		printf("Frequency: %8.1f\n", phase->power.frequency);

//		printf("Magnitude; Phase\n");
//		for (i = 0; i < N_ARRAY_LENGTH / 2; i++) {
//			sprintf(buff, "%12.4f %12.4f %12.4f %12.4f\n", phase->voltage.y_cf[i], phase->voltage.y_cf[N_ARRAY_LENGTH + i], phase->current.y_cf[i], phase->current.y_cf[N_ARRAY_LENGTH + i]);
//			printf("%s", buff);
//		}
//		printf("\n");
#elif 0
		for (i = 0 ; i < N_ARRAY_LENGTH / 2 ; i++) {
			phase->voltage.y_cf[i] = 10 * log10f((phase->voltage.y_cf[i * 2 + 0] * phase->voltage.y_cf[i * 2 + 0] + phase->voltage.y_cf[i * 2 + 1] * phase->voltage.y_cf[i * 2 + 1])/N_ARRAY_LENGTH);
			phase->current.y_cf[i] = 10 * log10f((phase->current.y_cf[i * 2 + 0] * phase->current.y_cf[i * 2 + 0] + phase->current.y_cf[i * 2 + 1] * phase->current.y_cf[i * 2 + 1])/N_ARRAY_LENGTH);
		}

		// Voltage
	    ESP_LOGW("FFT", "Voltage");
	    dsps_view(phase->voltage.y_cf, N_ARRAY_LENGTH / 2, 64, 10,  0, 90, '|');
		// Current
		ESP_LOGW("FFT", "Current");
		dsps_view(phase->current.y_cf, N_ARRAY_LENGTH / 2, 64, 10,  0, 90, '|');
#endif
//	    printf("FP = %.2f ", cos(phase->power.power_factor));
//	    if (phase->power.power_factor >= 0)
//	    {
//	    	printf("Ind\n");
//	    }
//	    else
//	    {
//	    	printf("Cap\n");
//	    }
		xTaskNotifyGiveIndexed(task_adc, INDEX_TO_WATCH);
	}
}
//// functions FFT


//// functions integration
void integration(float array[][3], int array_length, float *voltage_out, float *current_out)
{
#if _MECOEN_INTEGRATION_TYPE_ == (1 << 1) // Trapezoidal numerical integration method
/*
 * sum = (h/2) * [x(0) + 2 * x(1) + 2 * x(2) + ... + 2 * x(n - 2) + x(n - 1)]
*/
//		printf("\nTrapezoidal\n");
		*voltage_out = squared(array[1][0]);
		*current_out = squared(array[1][1]);

		for (int i = 2; i < array_length - 1; i++)
		{
			*voltage_out += squared(array[i][0]);
			*current_out += squared(array[i][1]);
		}

		*voltage_out += (squared(array[0][0]) + squared(array[array_length - 1][0])) / 2.0;
		*current_out += (squared(array[0][1]) + squared(array[array_length - 1][1])) / 2.0;

#elif _MECOEN_INTEGRATION_TYPE_ == (1 << 2) // Simpson numerical integration method
		// Requires even number of intervals, meaning array length with odd value and greater than 3
		// Array length has size 2^N, N integer, meaning even
		// Consider array length - 1 for Simpson and add last point considering Riemann sum trapezoidal
		// Initialize with first and second values of simpson method
		// initial value = 1st simpson      +         2nd simpson          + last value or simpson, array[N - 2]

//		printf("\nSimpson\n");
		*voltage_out = squared(array[0][0]) + (4.0 * squared(array[1][0])) + squared(array[array_length - 2][0]);
		*current_out = squared(array[0][1]) + (4.0 * squared(array[1][1])) + squared(array[array_length - 2][1]);

		for (int i = 2; i < array_length - 2; )
		{
			// Even positions
			*voltage_out += 2.0 * squared(array[i][0]);
			*current_out += 2.0 * squared(array[i][1]);

			i++;

			// Odd positions
			*voltage_out += 4.0 * squared(array[i][0]);
			*current_out += 4.0 * squared(array[i][1]);

			i++;
		}

		*voltage_out /= 3.0;
		*current_out /= 3.0;

		// Last point considering Reimann Trapezoidal sum
		// For RMS calculation will be divided by period, canceling the sampling_period_us
		*voltage_out += (squared(array[array_length - 2][0]) + squared(array[array_length - 2][0])) / 2.0;
		*current_out += (squared(array[array_length - 2][1]) + squared(array[array_length - 1][1])) / 2.0;

#else // riemann_rectangle
//	printf("\nRectangle\n");
	*voltage_out = squared(array[0][0]);
	*current_out = squared(array[0][1]);

	for (int i = 1; i < array_length; i++)
	{
		*voltage_out += squared(array[i][0]);
		*current_out += squared(array[i][1]);
	}
#endif
	// For RMS calculation will be divided by period, canceling the sampling_period_us
//	*voltage_out *= sampling_period_s;
//	*current_out *= sampling_period_s;
}
//// end functions integration
// end functions


#ifdef __cplusplus
extern "C"
#endif
void app_main()
{
	// Initializers
	//// Initializers SNTP
	/**
	 * NTP server address could be aquired via DHCP,
	 * see following menuconfig options:
	 * 'LWIP_DHCP_GET_NTP_SRV' - enable STNP over DHCP
	 * 'LWIP_SNTP_DEBUG' - enable debugging messages
	 *
	 * NOTE: This call should be made BEFORE esp aquires IP address from DHCP,
	 * otherwise NTP option would be rejected by default.
	 */
	#ifdef LWIP_DHCP_GET_NTP_SRV
		sntp_servermode_dhcp(1);      // accept NTP offers from DHCP server, if any
	#endif

	//// Initializers Semaphores
	semaphore_adc_main = xSemaphoreCreateBinary();
	semaphore_adc = xSemaphoreCreateBinary();

	//// Initializers Wi-Fi in Access Point + Station mode
	wifi_init_ap_sta();
	printf("\ninit wifi ap + sta\n");

	//// Initializers ADC
	init_adc();
	printf("\ninit_adc!\n");

	//// Initializers FFT
#if _MECOEN_FFT_
	ESP_ERROR_CHECK(init_fft());
	printf("\ninit_fft!\n");
#endif

	//// Initializers SNTP
	init_time();
#if _MECOEN_DS3231_
	char strftime_buf[64];
#endif
	struct tm timeinfo;
	timeval now;

	//// Initializers I2C | RTC DS3231
	gettimeofday(&now, NULL);
	localtime_r(&now.tv_sec, &timeinfo);
#if _MECOEN_DS3231_
	init_rtc_ds3231(&timeinfo);
#endif
	// end Initializers


	// Create Tasks
	//// Create Tasks Web server
	xTaskCreatePinnedToCore(&http_server, "http_server", 2048, NULL, 5, NULL, 1);

	//// Create Tasks ADC
    xTaskCreatePinnedToCore(read_phase, "read_phase", 2048, &phase_a, 5, &task_adc, 0);
    //// Create Tasks FFT
#if _MECOEN_FFT_
    xTaskCreatePinnedToCore(fft_continuous, "fft_continuous", 2048, &phase_a, 5, &task_fft, 0);
#endif
	//// Create Tasks I2C | RTC DS3231
#if _MECOEN_DS3231_
    xTaskCreate(rtc_ds3231, "rtc_ds3231", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
#endif
	// end Create Tasks

    printf("\nMain loop initialized!\n");
    while (1)
    {
#if _MECOEN_DS3231_
		gettimeofday(&now, NULL);
		localtime_r(&now.tv_sec, &timeinfo);
		strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
		printf("The current date/time is: %s\n", strftime_buf);
#endif

		// Copy data to copy array
		xSemaphoreTake(semaphore_adc_main, 10 * ticks_1s);
		for (int i = 0; i < n_array_copy_length; i++)
		{
			phase_copy[i][0] = phase_a.voltage.samples[i] - zmpt101b_vdc;
			phase_copy[i][1] = phase_a.current.samples[i] - sct013_vdc;
//			phase_copy[i][2] = phase_copy[i][0] * phase_copy[i][1];
		}
		xSemaphoreGive(semaphore_adc);
		// end data copy

		// RMS
		integration(phase_copy, n_array_copy_length, &phase_a.voltage.rms, &phase_a.current.rms);
		phase_a.voltage.rms = sqrt(phase_a.voltage.rms / n_array_copy_length);
		phase_a.current.rms = sqrt(phase_a.current.rms / n_array_copy_length);
		phase_a.power.apparent = phase_a.voltage.rms * phase_a.current.rms;
		// end RMS

#if 0
		for (int i = 0; i < n_array_copy_length; i++)
		{
			printf("%08.4f %08.4f %08.4f\n", phase_copy[i][0], phase_copy[i][1], phase_copy[i][2]);
			fflush(stdout);
			vTaskDelay(10 / portTICK_RATE_MS);
		}
		printf("\nVrms = %06.2f; Irms = %06.2f; P = %06.2f\n", phase_a.voltage.rms, phase_a.current.rms, phase_a.power.apparent);
		printf("\n\n");
		fflush(stdout);
#endif


		phase_a.voltage.rms = phase_a.voltage.rms_previous;
		phase_a.current.rms = phase_a.current.rms_previous;
    }

    // Delete Semaphores
    vSemaphoreDelete(semaphore_adc_main);
    vSemaphoreDelete(semaphore_adc);
}
