/*
 * Final project to achieve degree in Electrical Engineering with Emphasis in Electronic Systems at University of São Paulo
 * Project: Real Time Home Energy Meter with Web Application Interface
 * Author: Marcus Vinicius Gonçalves Mendes
 * Advisors: Roseli de Deus Lopes (EPUSP), Marcelo Knörich Zuffo (EPUSP), Alberto Ferreira de Sousa (UFES)
 *
*/

/*
 * Access point adapted from:
 * https://github.com/espressif/esp-idf/tree/v4.3/examples/wifi/getting_started/softAP
*/

/*
 * Web server adapted from:
 * https://github.com/caiomb/esp32-http_webserver
*/

#include <stdio.h>
#include <stdlib.h>
//#include <string.h>

#include "esp_bit_defs.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "soc/gpio_reg.h"
#include "soc/timer_group_reg.h"
#include "soc/timer_group_struct.h"

#include "esp_event.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "definitions.h"
#include "server.h"
#include "energy_meter_adc.h"
#include "energy_meter_time.h"
#include "energy_meter_fft.h"

// NVS related functions
static esp_err_t
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


// FFT
float wind[SAMPLING_FREQUENCY];


esp_err_t
init_fft()
{
	esp_err_t ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
	if (ret  != ESP_OK)
	{
		ESP_LOGE("FFT", "Not possible to initialize FFT. Error = %i", ret);
		return ret;
	}
    dsps_wind_hann_f32(wind, SAMPLING_FREQUENCY);
	// Generate hann window

    return ret;
}


void
fft_function(Signal *signal)
{
	// FFT
    dsps_fft2r_fc32(signal->y_cf, SAMPLING_FREQUENCY);
    // Bit reverse
	dsps_bit_rev_fc32(signal->y_cf, SAMPLING_FREQUENCY);
	// Convert one complex vector to two complex vectors
	dsps_cplx2reC_fc32(signal->y_cf, SAMPLING_FREQUENCY);
}

// end FFT


Circuit_phase phase_a;

static const int REASON = 4;
float phase_copy[SAMPLING_FREQUENCY/REASON][2];

#ifdef __cplusplus
extern "C"
#endif
void app_main()
{
	esp_err_t ret;
	init_adc();
	printf("\ninit_adc!\n");

	ret = init_fft();
	ESP_ERROR_CHECK(ret);
	printf("\ninit_fft!\n");

	ret = init_nvs();
    ESP_ERROR_CHECK(ret);
    printf("\ninit_nvs\n");

    http_server_setup();

    xTaskCreatePinnedToCore(read_phase, "read_phase", 2048, &phase_a, 5, NULL, 1);
    printf("\nread_phase task initialized!\n");

	vTaskDelay(500 / portTICK_RATE_MS);
    //int cnt = 0;

    printf("\nMain loop initialized!\n");
    while (1)
    {
		//cnt++;

		delayMicroseconds((int) 1e6);

		for (int i = 0; i < SAMPLING_FREQUENCY/REASON; i++)
		{
			phase_copy[i][0] = phase_a.voltage.samples[i];
			phase_copy[i][1] = phase_a.current.samples[i];
		}

		for (int i = 0; i < SAMPLING_FREQUENCY/REASON; i++)
		{
			printf("%07.2f %07.2f\n", phase_copy[i][0], phase_copy[i][1]);
			fflush(stdout);
			vTaskDelay(10 / portTICK_RATE_MS);
		}
		printf("\nVrms = %06.2f; Irms = %06.2f; P = %06.2f\n", phase_a.voltage.rms_previous, phase_a.current.rms_previous, phase_a.power.rms_previous);

		printf("\n\n");
		fflush(stdout);
    }
}
