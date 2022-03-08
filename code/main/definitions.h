/*
 * common_definitions.h
 *
 *  Created on: 5 de mar. de 2022
 *      Author: marcus
 */

#ifndef ENERGY_METER_PROJECT_DEFINITIONS_H_
#define ENERGY_METER_PROJECT_DEFINITIONS_H_

#include <math.h>
#include "esp_adc_cal.h"
#include "driver/adc.h"
#include "driver/gpio.h"


//// ADC definitions
#define SAMPLING_FREQUENCY 2500 // Number of samples per second
//#define V_SAMPLES		2500		// Number of voltage samples

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   4          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel_v = ADC_CHANNEL_6;     // GPIO34
static const adc_channel_t channel_i = ADC_CHANNEL_7;	  // GPIO35
static const adc_atten_t atten = ADC_ATTEN_DB_11; // max = 3.9V -> https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/adc.html
static const adc_unit_t unit = ADC_UNIT_1;

//#define V_SAMPLES		2500		// Number of voltage samples
#define SAMPLING_PERIOD_MS 1
static const uint32_t SAMPLING_PERIOD_US = 1e6 / SAMPLING_FREQUENCY; // Real sampling frequency slightly lower than 1e6/SAMPLING_PERIOD_US
static const float signal_to_rms = 1 / (SAMPLING_FREQUENCY * SAMPLING_PERIOD_US);

// voltage sensor parameters
#define ZMPT101B_VCC            3.3
#define ZMPT101B_R1			   9840 // 10k
#define ZMPT101B_R2			   9970 // 10k
/*\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/*/
#define ZMPT101B_VMAX		 0.8202 /* Calibrar /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\*/
/*\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/*/
static const float zmpt101b_dc_bias = 856/*(ZMPT101B_VCC * 1000 / 2) * ZMPT101B_R2 / (ZMPT101B_R1 + ZMPT101B_R2)*/; // Calculated: 1.107 V Measured: 1217 mV
static const float zmpt101b_calibration = (220 * sqrt(2)) / (ZMPT101B_VMAX * 1000 / 2);
// end voltage sensor parameters

// current sensor parameters
#define SCT013_VCC 				3.3
#define SCT013_NUMBER_TURNS    2000
#define SCT013_BURDEN_RESISTOR  466 // 470
#define SCT013_R1             21700 // 22k
#define SCT013_R2              9870 // 10k
static const float sct013_dc_bias = 1083;/*SCT013_VCC * 1000 * SCT013_R2 / (SCT013_R1 + SCT013_R2);*/ // Calculated: 1,032 V Measured: 1130 mV
static const float sct013_calibration = (SCT013_NUMBER_TURNS / SCT013_BURDEN_RESISTOR);
// end current sensor parameters
//// end ADC definitions


//// storage time
#define STORAGE_PERIOD 5 // minutes
#define STORAGE_MAiNTAIN 63 // days
//// end storage time

//// structures
typedef struct Signal
{
	float samples[SAMPLING_FREQUENCY];
	float rms_previous = 0, rms = 0;
} Signal;

typedef struct Circuit_phase
{
	Signal voltage, current, power;
} Circuit_phase;
//// end structures

#endif /* MAIN_DEFINITIONS_H_ */
