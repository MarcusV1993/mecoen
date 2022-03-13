/*
 * common_definitions.h
 *
 *  Created on: 5 de mar. de 2022
 *      Author: marcus
 */

#ifndef ENERGY_METER_PROJECT_DEFINITIONS_H_
#define ENERGY_METER_PROJECT_DEFINITIONS_H_

//// ADC definitions
#define SAMPLING_FREQUENCY 2048 // Number of samples per second | Used to set array length

//// storage
// storage time
#define STORAGE_PERIOD 5 // minutes
#define STORAGE_MAiNTAIN 63 // days
static const int storage_period_s = STORAGE_PERIOD * 60;
// end storage time
//// end storage

// structures
typedef struct Signal
{
	__attribute__((aligned(16)))
	float samples[SAMPLING_FREQUENCY];
	float rms_previous = 0, rms = 0;

	// FFT
	// working complex array
	__attribute__((aligned(16)))
	float y_cf[SAMPLING_FREQUENCY*2];
	// Pointers to result arrays
	float* y1_cf = &y_cf[0];
	float* y2_cf = &y_cf[SAMPLING_FREQUENCY];
	// Sum of y1 and y2
	__attribute__((aligned(16)))
	float sum_y[SAMPLING_FREQUENCY/2];
} Signal;


typedef struct Circuit_phase
{
	Signal voltage, current, power;
	float power_apparent, power_active, power_reactive, power_factor, freq;
} Circuit_phase;
// end structures

#endif /* MAIN_DEFINITIONS_H_ */
