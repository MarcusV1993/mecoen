/*
 * energy_meter_fft.h
 *
 *  Created on: 13 de mar. de 2022
 *      Author: marcus
 */

#ifndef MAIN_ENERGY_METER_FFT_H_
#define MAIN_ENERGY_METER_FFT_H_


#include "esp_dsp.h" // https://github.com/espressif/esp-dsp

#include "definitions.h"

//https://github.com/espressif/esp-dsp/blob/master/examples/fft/main/dsps_fft_main.c
// Window coefficients
__attribute__((aligned(16)))
extern float wind[SAMPLING_FREQUENCY];

#endif /* MAIN_ENERGY_METER_FFT_H_ */
