/*
 * energy_meter_adc.cpp
 *
 *  Created on: 13 de mar. de 2022
 *      Author: marcus
 */

#include <math.h>
#include <stdint.h>
#include "esp_adc_cal.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "mecoen_adc.h"

#include "mecoen_definitions.h"
#include "mecoen_time.h"

// ADC period
static const uint32_t SAMPLING_PERIOD_US = 1e6 / SAMPLING_FREQUENCY; // Real sampling frequency slightly lower than 1e6/SAMPLING_PERIOD_US
static const float signal_to_rms = 1 / (SAMPLING_FREQUENCY * SAMPLING_PERIOD_US);
// end ADC period


// ADC ports and configuration
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel_v = ADC_CHANNEL_6;     // GPIO34
static const adc_channel_t channel_i = ADC_CHANNEL_7;	  // GPIO35
static const adc_atten_t atten = ADC_ATTEN_DB_11; // max = 3.9V -> https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/adc.html
static const adc_unit_t unit = ADC_UNIT_1;
// end ADC ports and configuration


// voltage sensor
static const float zmpt101b_dc_bias = 1139/*(ZMPT101B_VCC * 1000 / 2) * ZMPT101B_R2 / (ZMPT101B_R1 + ZMPT101B_R2)*/; // Calculated: 1.107 V Measured: 1217 mV
static const float zmpt101b_calibration = (220 * sqrt(2)) / (ZMPT101B_VMAX * 1000 / 2);
// end voltage sensor


// current sensor
static const float sct013_dc_bias = 1025;/*SCT013_VCC * 1000 * SCT013_R2 / (SCT013_R1 + SCT013_R2);*/ // Calculated: 1,032 V Measured: 1130 mV
static const float sct013_calibration = (SCT013_NUMBER_TURNS / SCT013_BURDEN_RESISTOR);
// end current sensor

// functions
static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}


static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}


void
init_adc()
{
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

	// Init breaks pontentiometer sensor (https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/adc.html)
    // Usamos a ADC_UNIT_1

	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten((adc1_channel_t) channel_v, atten);
	adc1_config_channel_atten((adc1_channel_t) channel_i, atten); // Second ADC channel

    //Characterize ADC
    adc_chars = (esp_adc_cal_characteristics_t *) calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
    printf("\nadc initialized!\n");
}


double
get_adc1_value(adc_channel_t adc_channel)
{
	uint32_t adc_reading = 0;

	// Multisampling
	for (int i = 0; i < NO_OF_SAMPLES; i++)
		adc_reading += adc1_get_raw((adc1_channel_t) adc_channel);

	adc_reading /= NO_OF_SAMPLES;
	//Convert adc_reading to voltage in mV
	int voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

	return (voltage);
}


void
read_phase(void *arg)
{
	Circuit_phase *phase = (Circuit_phase *) arg;
	phase->voltage.rms = phase->voltage.rms_previous = 0.0;

//	uint32_t ret = 0;
	int sample_num = 0;

//	xTaskNotifyStateClearIndexed( NULL, indexToWaitOn );
//	configASSERT(task_fft != NULL);
//	xSemaphoreTake(semaphore_adc_fft, portMAX_DELAY);

	while(1)
	{
//		Reading ADC
		phase->voltage.samples[sample_num] = get_adc1_value(channel_v);
		delayMicroseconds(1);
		phase->current.samples[sample_num] = get_adc1_value(channel_i);

//		Remove DC bias, mV -> V
		phase->voltage.samples[sample_num] = (phase->voltage.samples[sample_num] - zmpt101b_dc_bias);
		phase->current.samples[sample_num] = (phase->current.samples[sample_num] - sct013_dc_bias);

//		Convert mV -> V
//		phase->voltage.samples[sample_num] /= 1000;
//		phase->current.samples[sample_num] /= 1000;

//		Convert ADC readings to real world value
//		phase->voltage.samples[sample] = zmpt101b_calibration * (phase->voltage.samples[sample] - zmpt101b_dc_bias);
//		phase->current.samples[sample] =   sct013_calibration * (phase->current.samples[sample] -   sct013_dc_bias);

//		Instant power
		phase->power.samples[sample_num] = phase->voltage.samples[sample_num] * phase->current.samples[sample_num];

//		RMS
		phase->voltage.rms += phase->voltage.samples[sample_num]*phase->voltage.samples[sample_num];
		phase->current.rms += phase->current.samples[sample_num]*phase->current.samples[sample_num];

		sample_num++;
		if(sample_num >= SAMPLING_FREQUENCY)
		{
			sample_num = 0;

			xSemaphoreGive(semaphore_fft);
//			xTaskNotifyGiveIndexed(task_fft, indexToWaitOn); // Notify fft task that array has been filled
//printf("\nADC give task\n");
//delayMicroseconds(100);
			xSemaphoreTake(semaphore_adc, portMAX_DELAY);
//			ulTaskNotifyTakeIndexed(indexToWaitOn, pdFALSE, portMAX_DELAY/*pdMS_TO_TICKS(5000)*/); // Wait until fft task finished copying array
//printf("\nADC take task\n");

//	Imprecision for not calculating RMS considering integer multiples of the signal period
			phase->voltage.rms_previous = sqrt(phase->voltage.rms / SAMPLING_FREQUENCY);
			phase->current.rms_previous = sqrt(phase->current.rms / SAMPLING_FREQUENCY);

			phase->power.rms_previous = phase->voltage.rms_previous * phase->current.rms_previous;

			phase->voltage.rms = 0.0;
			phase->current.rms = 0.0;
		}

		delayMicroseconds(SAMPLING_PERIOD_US);
	}
}
// end functions
