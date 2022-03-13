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

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "definitions.h"
#include "server.h"
#include "energy_meter_adc.h"
#include "energy_meter_time.h"
#include "energy_meter_fft.h"


Circuit_phase phase_a;


static const int REASON = 4;
float phase_copy[SAMPLING_FREQUENCY/REASON][2];


#ifdef __cplusplus
extern "C"
#endif
void app_main()
{
	init_adc();
	printf("\ninit_adc!\n");

	ESP_ERROR_CHECK(init_fft());
	printf("\ninit_fft!\n");

    http_server_setup();

    xTaskCreatePinnedToCore(read_phase, "read_phase", 2048, &phase_a, 5, NULL, 1);
    printf("\nread_phase task initialized!\n");

	vTaskDelay(500 / portTICK_RATE_MS);

    printf("\nMain loop initialized!\n");
    while (1)
    {
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
