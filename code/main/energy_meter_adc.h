/*
 * energy_meter_adc.h
 *
 *  Created on: 13 de mar. de 2022
 *      Author: marcus
 */

#ifndef ENRGY_METER_PROJECT_ADC_FUNCTIONS_H_
#define ENRGY_METER_PROJECT_ADC_FUNCTIONS_H_

#include "energy_meter_adc.h"

#include <math.h>

#include "definitions.h"
#include "esp_adc_cal.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "energy_meter_time.h"

//#define SAMPLING_FREQUENCY 2048 // Number of samples per second

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   4          //Multisampling

//static const uint32_t SAMPLING_PERIOD_US = 1e6 / SAMPLING_FREQUENCY; // Real sampling frequency slightly lower than 1e6/SAMPLING_PERIOD_US
//static const float signal_to_rms = 1 / (SAMPLING_FREQUENCY * SAMPLING_PERIOD_US);
//
//static esp_adc_cal_characteristics_t *adc_chars;
//static const adc_channel_t channel_v = ADC_CHANNEL_6;     // GPIO34
//static const adc_channel_t channel_i = ADC_CHANNEL_7;	  // GPIO35
//static const adc_atten_t atten = ADC_ATTEN_DB_11; // max = 3.9V -> https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/adc.html
//static const adc_unit_t unit = ADC_UNIT_1;


// voltage sensor parameters
#define ZMPT101B_VCC            3.3
#define ZMPT101B_R1			   9840 // 10k
#define ZMPT101B_R2			   9970 // 10k

#define ZMPT101B_VMAX		 0.8202 // Calibrar
// end voltage sensor parameters

// current sensor parameters
#define SCT013_VCC 				3.3
#define SCT013_NUMBER_TURNS    2000
#define SCT013_BURDEN_RESISTOR  466 // 470
#define SCT013_R1             21700 // 22k
#define SCT013_R2              9870 // 10k
// end current sensor parameters

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

// accessible ADC functions
void init_adc();

void read_phase(void *arg);
// end accessible ADC functions

#endif /* ENRGY_METER_PROJECT_ADC_FUNCTIONS_H_ */
