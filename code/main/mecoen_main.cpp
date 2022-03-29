/*
 * Final project to achieve degree in Electrical Engineering with Emphasis in Electronic Systems at University of São Paulo
 * Project: Real Time Home Energy Meter with Web Application Interface
 * Author: Marcus Vinicius Gonçalves Mendes
 * Advisors: Roseli de Deus Lopes (EPUSP), Marcelo Knörich Zuffo (EPUSP), Alberto Ferreira de Sousa (UFES)
 *
*/

/*
 * Web server adapted from:
 * https://github.com/caiomb/esp32-http_webserver
*/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_sntp.h"

#include "mecoen_adc.h"
#include "mecoen_wifi.h"
#include "server.h"
#include "mecoen_commons.h"
#include "mecoen_definitions.h"
#include "mecoen_fft.h"
#include "mecoen_time.h"
#include "mecoen_i2c.h"


Circuit_phase phase_a;


static const int REASON = 4;
float phase_copy[SAMPLING_FREQUENCY/REASON][3];

TaskHandle_t task_adc = NULL, task_fft = NULL;
SemaphoreHandle_t semaphore_adc = xSemaphoreCreateMutex();
SemaphoreHandle_t semaphore_fft = xSemaphoreCreateMutex();

#ifdef __cplusplus
extern "C"
#endif
void app_main()
{
	// Initializers
		// SNTP
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

		// Initialize Wi-Fi in Access Point + Station mode
	wifi_init_ap_sta();

		// Circuit phases
	init_phase(&phase_a);

		// ADC
	init_adc();
	printf("\ninit_adc!\n");

		// FFT
	ESP_ERROR_CHECK(init_fft());
	printf("\ninit_fft!\n");

		// SNTP
	init_time();
	char strftime_buf[64];
	struct tm timeinfo;
	timeval now;

		// I2C | RTC DS3231
	gettimeofday(&now, NULL);
	localtime_r(&now.tv_sec, &timeinfo);
	init_rtc_ds3231(&timeinfo);
	// end Initializers

	// Take semaphore to sync FFT and ADC tasks
//	xSemaphoreTake(semaphore_fft, portMAX_DELAY);
//	xSemaphoreTake(semaphore_adc, portMAX_DELAY);

	// Create Tasks
		// Web server
	xTaskCreate(&http_server, "http_server", 2048, NULL, 5, NULL);

		// ADC
    xTaskCreatePinnedToCore(read_phase, "read_phase", 2048, &phase_a, 5, &task_adc, 1);
    printf("\nread_phase task initialized!\n");

    	// FFT
//    xTaskCreatePinnedToCore(fft_continuous, "fft_continuous", 2048, &phase_a, 5, &task_fft, 1);
//	printf("\nread_fft task initialized!\n");

		// I2C | RTC DS3231
//    xTaskCreate(rtc_ds3231, "rtc_ds3231", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
	// end Create Tasks

	vTaskDelay(500 / portTICK_RATE_MS);

    printf("\nMain loop initialized!\n");
    while (1)
    {
		delayMicroseconds((int) 1e6); // 5 s
//        vTaskDelay(pdMS_TO_TICKS(5000)); // 5 s

//		gettimeofday(&now, NULL);
//		localtime_r(&now.tv_sec, &timeinfo);
//		strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
//		printf("The current date/time is: %s\n", strftime_buf);
		for (int i = 0; i < SAMPLING_FREQUENCY/REASON; i++)
		{
			phase_copy[i][0] = (phase_a.voltage.samples[i] - 1243 + 50) / 1000;
			phase_copy[i][1] = (phase_a.current.samples[i] - 1060) / 1000;
			phase_copy[i][2] = phase_copy[i][0] * phase_copy[i][1]; //phase_a.power.samples[i];
		}

		for (int i = 0; i < SAMPLING_FREQUENCY/REASON; i++)
		{
			printf("%06.4f %06.4f %04.4f\n", phase_copy[i][0], phase_copy[i][1], phase_copy[i][2]);
			fflush(stdout);
			vTaskDelay(10 / portTICK_RATE_MS);
		}
//		printf("\nVrms = %06.2f; Irms = %06.2f; P = %06.2f\n", phase_a.voltage.rms_previous, phase_a.current.rms_previous, phase_a.power.rms_previous);

		printf("\n\n");
		fflush(stdout);
    }
}
