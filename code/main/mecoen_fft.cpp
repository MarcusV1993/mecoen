/*
 * energy_meter_fft.cpp
 *
 *  Created on: 13 de mar. de 2022
 *      Author: marcus
 */

#include <stdint.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_dsp.h" // https://github.com/espressif/esp-dsp

#include "mecoen_fft.h"
#include "mecoen_definitions.h"

//https://github.com/espressif/esp-dsp/blob/master/examples/fft/main/dsps_fft_main.c
// Window coefficients
__attribute__((aligned(16)))
float wind[N_ARRAY_LENGTH]; // ?? extern float wind[SAMPLING_FREQUENCY]; ??

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

	signal->mag_phase.mag = 0.0;
	int max_magnitude_index = 0;
	float max_magnitude = 0.0, magnitude_k = 0.0;
	for (int k = 0; k < N_ARRAY_LENGTH / 2; k++)
	{
		magnitude_k = ((signal->y_cf[k * 2] * signal->y_cf[k * 2]) + (signal->y_cf[(k * 2) + 1] * signal->y_cf[(k * 2) + 1])) / SAMPLING_FREQUENCY;
		signal->mag_phase.mag += magnitude_k;
		if (magnitude_k > max_magnitude)
		{
			max_magnitude = magnitude_k;
			max_magnitude_index = k;
		}
        signal->y_cf[k] = 10 * log10f(magnitude_k);
	}
	signal->mag_phase.phase_max_mag = atan2((double) signal->y_cf[max_magnitude_index + 1], (double) signal->y_cf[max_magnitude_index]);
}

void
fft_circuit_phase(Circuit_phase *phase)
{
	// Voltage, Current, Power
	int   max_magnitude_index[3] = {  0,   0,   0};
	float       max_magnitude[3] = {0.0, 0.0, 0.0};
	float         magnitude_k[3] = {0.0, 0.0, 0.0};

	// Voltage
	fft_function(&phase->voltage);

	// Current
	fft_function(&phase->current);

	// Power
	fft_function(&phase->power);

	phase->voltage.mag_phase.mag = 0.0;
	phase->current.mag_phase.mag = 0.0;
	  phase->power.mag_phase.mag = 0.0;
	for (int k = 0; k < N_ARRAY_LENGTH / 2; k++) {
		// Voltage
		magnitude_k[0] = ((phase->voltage.y_cf[k * 2] * phase->voltage.y_cf[k * 2]) + (phase->voltage.y_cf[(k * 2) + 1] * phase->voltage.y_cf[(k * 2) + 1])) / SAMPLING_FREQUENCY;
		phase->voltage.mag_phase.mag += magnitude_k[0];
		if (magnitude_k[0] > max_magnitude[0])
		{
			max_magnitude[0] = magnitude_k[0];
			max_magnitude_index[0] = k;
		}
		phase->voltage.y_cf[k] = 10 * log10f(magnitude_k[0]);

		// Current
		magnitude_k[1] = ((phase->current.y_cf[k * 2] * phase->current.y_cf[k * 2]) + (phase->current.y_cf[(k * 2) + 1] * phase->current.y_cf[(k * 2) + 1])) / SAMPLING_FREQUENCY;
		phase->current.mag_phase.mag += magnitude_k[1];
		if (magnitude_k[1] > max_magnitude[1])
		{
			max_magnitude[1] = magnitude_k[1];
			max_magnitude_index[1] = k;
		}
		phase->current.y_cf[k] = 10 * log10f(magnitude_k[1]);

		// Power
		magnitude_k[2] = ((phase->power.y_cf[k * 2] * phase->power.y_cf[k * 2]) + (phase->power.y_cf[(k * 2) + 1] * phase->power.y_cf[(k * 2) + 1])) / SAMPLING_FREQUENCY;
		phase->power.mag_phase.mag += magnitude_k[2];
		if (magnitude_k[2] > max_magnitude[2])
		{
			max_magnitude[2] = magnitude_k[2];
			max_magnitude_index[2] = k;
		}
		phase->power.y_cf[k] = 10 * log10f(magnitude_k[2]);
	}
	// Voltage
	phase->voltage.mag_phase.phase_max_mag = atan2((double) phase->voltage.y_cf[max_magnitude_index[0] + 1], \
													(double) phase->voltage.y_cf[max_magnitude_index[0]]);

	// Current
	phase->current.mag_phase.phase_max_mag = atan2((double) phase->current.y_cf[max_magnitude_index[1] + 1], \
													(double) phase->current.y_cf[max_magnitude_index[1]]);

	// Power
	phase->power.mag_phase.phase_max_mag = atan2((double) phase->power.y_cf[max_magnitude_index[2] + 1], \
													(double) phase->power.y_cf[max_magnitude_index[2]]);

	phase->power_factor = phase->power.mag_phase.phase_max_mag - phase->voltage.mag_phase.phase_max_mag;
	if (phase->power_factor >= 2 * M_PI)
	{
		phase->power_factor -= 2 * M_PI;
	}
	else if (phase->power_factor <= -2 * M_PI)
	{
		phase->power_factor += 2 * M_PI;
	}
}

void
fft_continuous(void *arg)
{
	Circuit_phase *phase = (Circuit_phase *) arg;
//	uint32_t ret = 0;

	int i;

//	xTaskNotifyStateClearIndexed( NULL, indexToWaitOn );
//	configASSERT(task_adc != NULL);

	while(1)
	{
//		ret = ulTaskNotifyTakeIndexed(indexToWaitOn, pdFALSE, portMAX_DELAY/*pdMS_TO_TICKS(5000)*/); // Wait for adc task to fill array
		xSemaphoreTake(semaphore_fft, portMAX_DELAY);
//printf("\nFFT received task\n");
		for (i = 0; i < N_ARRAY_LENGTH; i++)
		{
			phase->voltage.y_cf[2 * i] = phase->voltage.samples[i] * wind[i];
			phase->voltage.y_cf[(2 * i) + 1] = 0; // Real signal has imaginary part 0

			phase->current.y_cf[2 * i] = phase->current.samples[i] * wind[i];
			phase->current.y_cf[(2 * i) + 1] = 0; // Real signal has imaginary part 0

			phase->power.y_cf[2 * i] = phase->power.samples[i] * wind[i];
			phase->power.y_cf[(2 * i) + 1] = 0; // Real signal has imaginary part 0
		}
		xSemaphoreGive(semaphore_adc);
//		xTaskNotifyGiveIndexed(task_adc, indexToWaitOn); // Notify adc task that array has been copied
//printf("\nFFT give task\n");

		fft_function(&phase->voltage);
//		printf("voltage Mag: %f Phase:%f\n", phase->voltage.mag_phase.mag, phase->voltage.mag_phase.phase_max_mag);
		fft_function(&phase->current);
//		printf("current Mag: %f Phase:%f\n", phase->current.mag_phase.mag, phase->current.mag_phase.phase_max_mag);
		// Voltage
//	    ESP_LOGW("FFT", "Voltage");
//	    dsps_view(phase->voltage.y_cf, SAMPLING_FREQUENCY / 2, 64, 10,  -60, 40, '|');
		// Current
//		ESP_LOGW("FFT", "Current");
//		dsps_view(phase->current.y_cf, SAMPLING_FREQUENCY / 2, 64, 10,  -60, 40, '|');
		// Power
//	    ESP_LOGW("FFT", "Power");
//		dsps_view(phase->power.y_cf, SAMPLING_FREQUENCY / 2, 64, 10,  -60, 40, '|');

//	    printf("FP = %.2f ", cos(phase->power_factor));
//	    if (phase->power_factor >= 0)
//	    {
//	    	printf("Ind\n");
//	    }
//	    else
//	    {
//	    	printf("Cap\n");
//	    }
		vTaskDelay(10);
	}
}
