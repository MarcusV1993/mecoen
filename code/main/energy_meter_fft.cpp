/*
 * energy_meter_fft.cpp
 *
 *  Created on: 13 de mar. de 2022
 *      Author: marcus
 */


#include "energy_meter_fft.h"

#include "esp_dsp.h" // https://github.com/espressif/esp-dsp

#include "definitions.h"

//https://github.com/espressif/esp-dsp/blob/master/examples/fft/main/dsps_fft_main.c
// Window coefficients
__attribute__((aligned(16)))
float wind[SAMPLING_FREQUENCY]; // ?? extern float wind[SAMPLING_FREQUENCY]; ??


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
