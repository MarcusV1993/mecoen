/*
 * common_definitions.h
 *
 *  Created on: 5 de mar. de 2022
 *      Author: marcus
 */

#ifndef ENERGY_METER_PROJECT_DEFINITIONS_H_
#define ENERGY_METER_PROJECT_DEFINITIONS_H_

#include <stdint.h>

//// ADC definitions
#define SAMPLING_FREQUENCY 2048 // Number of samples per second | Used to set array length
const int SAMPLING_FREQUENCY2 = 2 * SAMPLING_FREQUENCY;

//// storage
// storage time
#define STORAGE_PERIOD 5 // minutes
#define STORAGE_MAiNTAIN 63 // days
static const int storage_period_s = STORAGE_PERIOD * 60;
// end storage time
//// end storage

typedef struct Mag_phase
{
	float mag;
	float phase_max_mag;
} Mag_phase;

// structures
typedef struct Signal
{
	float samples[SAMPLING_FREQUENCY];
	float rms_previous = 0, rms = 0;

	// FFT
	// working complex array
	__attribute__((aligned(16)))
	float y_cf[SAMPLING_FREQUENCY*2];
	// Pointers to result arrays
	float* y1_cf = &y_cf[0];
	float* y2_cf = &y_cf[SAMPLING_FREQUENCY];
	Mag_phase mag_phase;
	// Sum of y1 and y2
//	__attribute__((aligned(16)))
//	float sum_y[SAMPLING_FREQUENCY/2];
} Signal;


typedef struct Circuit_phase
{
	Signal voltage, current, power;
	float power_apparent, power_active, power_reactive, power_factor, freq;
} Circuit_phase;
// end structures

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

extern TaskHandle_t task_fft, task_adc;
const UBaseType_t indexToWaitOn = 1;
extern SemaphoreHandle_t semaphore_adc_fft, semaphore_adc, semaphore_fft;

#endif /* MAIN_DEFINITIONS_H_ */
