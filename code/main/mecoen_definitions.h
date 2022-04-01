/*
 * common_definitions.h
 *
 *  Created on: 5 de mar. de 2022
 *      Author: marcus
 */

#ifndef ENERGY_METER_PROJECT_DEFINITIONS_H_
#define ENERGY_METER_PROJECT_DEFINITIONS_H_

#include <stdint.h>

#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
#include "freertos/semphr.h"


// Array size
#define N_ARRAY_LENGTH 2048
const int N_ARRAY_LENGTH2 = 2 * N_ARRAY_LENGTH;

//// ADC definitions
#define SAMPLING_FREQUENCY 2500
//// Initial DC value for ADC readings
#define ZMPT101B_VDC 1211.8
#define   SCT013_VDC 1257.45

#define ZMPT101B_CONSTANT_MULTIPLIER 0.60595
#define SCT013_CONSTANT_MULTIPLIER 0.00932
//// end Initial DC value for ADC readings
const int SAMPLING_FREQUENCY2 = 2 * SAMPLING_FREQUENCY;

//// to save moving average
extern float zmpt101b_vdc;
extern float  sct013_vdc;
////end to save moving average
// end ADC definitions

//// storage
// storage time
#define STORAGE_PERIOD 5 // minutes
#define STORAGE_MAINTAIN 63 // days
static const int storage_period_s = STORAGE_PERIOD * 60;
// end storage time
//// end storage


// structures
// Structure to store magnitude and phase of a signal
typedef struct Mag_phase
{
	float mag;
	float phase_max_mag;
} Mag_phase;

// Structure to store signal readings, as well as it's Root Mean Squared value, and the array for FFT calculations
typedef struct Signal
{
	float samples[N_ARRAY_LENGTH];
	float rms_previous = 0, rms = 0;

	// FFT
	// working complex array
	__attribute__((aligned(16)))
	float y_cf[N_ARRAY_LENGTH2];
	// Pointers to result arrays
	float* y1_cf = &y_cf[0];
	float* y2_cf = &y_cf[N_ARRAY_LENGTH];
	Mag_phase mag_phase;
	// Sum of y1 and y2
//	__attribute__((aligned(16)))
//	float sum_y[SAMPLING_FREQUENCY/2];
} Signal;

// Structure for voltage and current readings in a phase of the circuit
// Variables to store power of the phase
typedef struct Circuit_phase
{
	Signal voltage, current, power;
	float power_apparent, power_active, power_reactive, power_factor, freq;
} Circuit_phase;
// end structures


// Task sincronization variables
//extern TaskHandle_t task_fft, task_adc;
//const UBaseType_t indexToWaitOn = 1;
extern SemaphoreHandle_t semaphore_adc_dc, semaphore_adc, semaphore_fft;
// end Task sincronization variables

#endif /* MAIN_DEFINITIONS_H_ */
