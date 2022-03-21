/*
 * energy_meter_fft.h
 *
 *  Created on: 13 de mar. de 2022
 *      Author: marcus
 */

#ifndef MAIN_MECOEN_FFT_H_
#define MAIN_MECOEN_FFT_H_


#include "esp_err.h"

// functions
esp_err_t init_fft();

void fft_continuous(void *arg);
// end functions

#endif /* MAIN_MECOEN_FFT_H_ */
