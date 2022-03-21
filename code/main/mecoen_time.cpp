/*
 * energy_meter_time.cpp
 *
 *  Created on: 13 de mar. de 2022
 *      Author: marcus
 *
 *  some functions adapted from: https://github.com/espressif/esp-idf/tree/master/examples/protocols/sntp
 */

#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "esp_timer.h"
#include "driver/gptimer.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sntp.h"

#include "mecoen_definitions.h"
#include "mecoen_time.h"


/* Variable holding number of times ESP32 restarted since first boot.
 * It is placed into RTC memory using RTC_DATA_ATTR and
 * maintains its value when ESP32 wakes from deep sleep.
 */
RTC_DATA_ATTR static int boot_count = 0;

static const char *TAG = "mecoen_time";


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


double
ojGetTimeSec()
{
	int64_t t = esp_timer_get_time();
	double t_seconds = (double) t / 1000000.0;

	return (t_seconds);
}


void
ojSleepMsec(double miliseconds)
{
	double t = ojGetTimeSec();
    while ((ojGetTimeSec() - t) < (miliseconds / 1000.0));
}


/////////////////////////////////////////////////////////////
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_CUSTOM
void sntp_sync_time(struct timeval *tv)
{
   settimeofday(tv, NULL);
   ESP_LOGI(TAG, "Time is synchronized from custom code");
   sntp_set_sync_status(SNTP_SYNC_STATUS_COMPLETED);
}
#endif


void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}


static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
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

    ESP_LOGI(TAG, "List of configured NTP servers:");

    for (uint8_t i = 0; i < SNTP_MAX_SERVERS; ++i){
        if (sntp_getservername(i)){
            ESP_LOGI(TAG, "server %d: %s", i, sntp_getservername(i));
        } else {
            // we have either IPv4 or IPv6 address, let's print it
            char buff[INET6_ADDRSTRLEN];
            ip_addr_t const *ip = sntp_getserver(i);
            if (ipaddr_ntoa_r(ip, buff, INET6_ADDRSTRLEN) != NULL)
                ESP_LOGI(TAG, "server %d: %s", i, buff);
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
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
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
	ESP_LOGI(TAG, "Boot count: %d", boot_count);

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
		ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
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
			ESP_LOGI(TAG, "Waiting for adjusting time ... outdelta = %li sec: %li ms: %li us",
						(long)outdelta.tv_sec,
						outdelta.tv_usec/1000,
						outdelta.tv_usec%1000);
			vTaskDelay(2000 / portTICK_PERIOD_MS);
		}
	}
}

