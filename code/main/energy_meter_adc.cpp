/*
 * energy_meter_adc.cpp
 *
 *  Created on: 13 de mar. de 2022
 *      Author: marcus
 */

#include "energy_meter_adc.h"

static const uint32_t SAMPLING_PERIOD_US = 1e6 / SAMPLING_FREQUENCY; // Real sampling frequency slightly lower than 1e6/SAMPLING_PERIOD_US
static const float signal_to_rms = 1 / (SAMPLING_FREQUENCY * SAMPLING_PERIOD_US);

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel_v = ADC_CHANNEL_6;     // GPIO34
static const adc_channel_t channel_i = ADC_CHANNEL_7;	  // GPIO35
static const adc_atten_t atten = ADC_ATTEN_DB_11; // max = 3.9V -> https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/adc.html
static const adc_unit_t unit = ADC_UNIT_1;

// voltage sensor
static const float zmpt101b_dc_bias = 856/*(ZMPT101B_VCC * 1000 / 2) * ZMPT101B_R2 / (ZMPT101B_R1 + ZMPT101B_R2)*/; // Calculated: 1.107 V Measured: 1217 mV
static const float zmpt101b_calibration = (220 * sqrt(2)) / (ZMPT101B_VMAX * 1000 / 2);

// current sensor
static const float sct013_dc_bias = 1083;/*SCT013_VCC * 1000 * SCT013_R2 / (SCT013_R1 + SCT013_R2);*/ // Calculated: 1,032 V Measured: 1130 mV
static const float sct013_calibration = (SCT013_NUMBER_TURNS / SCT013_BURDEN_RESISTOR);

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

	int sample = 0;

	while(1)
	{
//		Reading ADC
		phase->voltage.samples[sample] = get_adc1_value(channel_v);
		delayMicroseconds(1);
		phase->current.samples[sample] = get_adc1_value(channel_i);

//		Convert ADC readings to real world value
//		phase->voltage.samples[sample] = zmpt101b_calibration * (phase->voltage.samples[sample] - zmpt101b_dc_bias);
//		phase->current.samples[sample] =   sct013_calibration * (phase->current.samples[sample] -   sct013_dc_bias);

//		Instant power
		phase->power.samples[sample] = phase->voltage.samples[sample] * phase->current.samples[sample];

//		RMS
		phase->voltage.rms += phase->voltage.samples[sample]*phase->voltage.samples[sample];
		phase->current.rms += phase->current.samples[sample]*phase->current.samples[sample];

		sample++;
		if(sample >= SAMPLING_FREQUENCY)
		{
			sample = 0;
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

