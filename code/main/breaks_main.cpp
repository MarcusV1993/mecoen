/*
 *  To-do: Convert voltage from read_voltage(void *arg) to real world voltage
 *  To-do: Convert voltage from read_current_voltage(void *arg) to real world current
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h> // Marcus include
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "soc/gpio_reg.h"
#include "soc/timer_group_reg.h"
#include "soc/timer_group_struct.h"
#include "esp_bit_defs.h"
#include "driver/gptimer.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "esp_timer.h"

#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"


#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   4          //Multisampling

#define V_SAMPLES		2500		// Number of voltage samples
#define SAMPLING_PERIOD_MS 1
static const uint32_t SAMPLING_PERIOD_US = 1e3/V_SAMPLES; // Real sampling frequency slightly lower than 1e6/SAMPLING_PERIOD_US
static const float signal_to_rms = 1 / (V_SAMPLES * (1000 * SAMPLING_PERIOD_MS));

#define V0 1110.8 //1110.828
#define I0 1087.5 //1087.484

// circuit parameters
#define VCC             5
// end circuit parameters

// current sensor parameters
#define sct013_VCC 				3.3
#define SCT013_NUMBER_TURNS    2000
#define SCT013_BURDEN_RESISTOR  470
#define SCT013_R1              22e3
#define SCT013_R2              10e3
static const float sct013_dc_bias = VCC * SCT013_R2 / (SCT013_R1 + SCT013_R2);
static const float sct013_calibration = SCT013_NUMBER_TURNS / sct013_dc_bias;
// end current sensor parameters

//
typedef struct Signal
{
	float samples[V_SAMPLES];
	float rms_previous, rms;
} Signal;

typedef struct Circuit_phase
{
	Signal voltage, current, power;
} Circuit_phase;

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2 -> o Potenciomentro vai no GPIO34
static const adc_atten_t atten = ADC_ATTEN_DB_11; // max = 3.9V -> https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/adc.html
static const adc_unit_t unit = ADC_UNIT_1;

static const adc_channel_t channel7 = ADC_CHANNEL_7;


#define NOP() asm volatile ("nop")

unsigned long IRAM_ATTR micros()
{
    return (unsigned long) (esp_timer_get_time());
}
void IRAM_ATTR delayMicroseconds(uint32_t us)
{
    uint32_t m = micros();
    if(us){
        uint32_t e = (m + us);
        if(m > e){ //overflow
            while(micros() > e){
                NOP();
            }
        }
        while(micros() < e){
            NOP();
        }
    }
}


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


double
ojGetTimeSec()
{
	int64_t t = esp_timer_get_time();
	double t_seconds = (double) t / 1000000.0;

	return (t_seconds);
}


void
ojSleepMsec(double miliseconds)
{
	double t = ojGetTimeSec();
    while ((ojGetTimeSec() - t) < (miliseconds / 1000.0));
}


double
get_voltage()
{
    uint32_t adc_reading = 0;
    //Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++)
    {
        if (unit == ADC_UNIT_1) // Usamos a ADC_UNIT_1
        {
            adc_reading += adc1_get_raw((adc1_channel_t) channel);
        }
        else
        {
            int raw;
            adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
            adc_reading += raw;
        }
    }
    adc_reading /= NO_OF_SAMPLES;
    //Convert adc_reading to voltage in mV
    int voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

    return (voltage);
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
init_adc()
{
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

	// Init breaks pontentiometer sensor (https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/adc.html)
    if (unit == ADC_UNIT_1) // Usamos a ADC_UNIT_1
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten((adc1_channel_t) channel, atten);
        adc1_config_channel_atten((adc1_channel_t) channel7, atten); // Second ADC channel
    }
    else
        adc2_config_channel_atten((adc2_channel_t) channel, atten);

    //Characterize ADC
    adc_chars = (esp_adc_cal_characteristics_t *) calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
    printf("\nadc initialized!\n");
}


static void
read_voltage(void *arg)
{
	Signal *voltage = (Signal *) arg;

	voltage->rms = 0.0;
	voltage->rms_previous = 0.0;

	int sample = 0;

	while (1)
	{
	    voltage->samples[sample] = get_voltage();
	    voltage->rms += voltage->samples[sample] * voltage->samples[sample];
		sample++;
		if (sample >= V_SAMPLES)
		{
			sample = 0;
			voltage->rms_previous = sqrtf(voltage->rms * signal_to_rms);
			voltage->rms = 0.0;
		}

		delayMicroseconds(100);
//		vTaskDelay(1 / portTICK_RATE_MS);
	}
}


void
read_current(void *arg)
{
	Signal *current = (Signal *) arg;
	current->rms = 0.0;
	current->rms_previous = 0.0;

	int sample = 0;

	while (1)
	{
		current->samples[sample] = get_adc1_value(channel7);
//		Test:
//		current->samples[sample] = sct013_calibration * (get_voltage() - sct013_dc_bias)
		current->rms += current->samples[sample];
		sample++;
		if (sample >= V_SAMPLES)
		{
			sample = 0;
			current->rms_previous = sqrt(current->rms);
			current->rms = 0.0;
		}
//		ojSleepMsec(1.0);
		vTaskDelay(1 / portTICK_RATE_MS);
	}
}

void
read_phase(void *arg)
{
	Circuit_phase *phase = (Circuit_phase *) arg;
	phase->voltage.rms = phase->voltage.rms_previous = 0.0;

	int sample = 0;

	while(1)
	{
		phase->voltage.samples[sample] = get_voltage() - V0;
		phase->current.samples[sample] = get_adc1_value(channel7) - I0;

//		Convert adc readings to real world value

		phase->voltage.rms += phase->voltage.samples[sample]*phase->voltage.samples[sample];
		phase->current.rms += phase->current.samples[sample]*phase->current.samples[sample];

		sample++;
		if(sample >= V_SAMPLES)
		{
			sample = 0;
//	Imprecision for not calculating rms considering integer multiples of the signal period
			phase->voltage.rms_previous = sqrt(phase->voltage.rms);
			phase->current.rms_previous = sqrt(phase->current.rms);

			phase->voltage.rms = 0.0;
			phase->current.rms = 0.0;
		}

		delayMicroseconds(SAMPLING_PERIOD_US);
	}
}

//Signal voltage;
//Signal current;
Circuit_phase phase_a;
//float vcopy[V_SAMPLES];
static const int REASON = 4;
float phase_copy[V_SAMPLES/REASON][2];

#ifdef __cplusplus
extern "C"
#endif
void app_main()
{
	init_adc();

//    printf("\nCreated Signal voltage\n");
//    xTaskCreatePinnedToCore(read_voltage, "read_voltage", 2048, &voltage, 5, NULL, 1);
//    printf("\nread_voltage task initialized!\n");

    xTaskCreatePinnedToCore(read_phase, "read_phase", 2048, &phase_a, 5, NULL, 1);
    printf("\nread_phase task initialized!\n");

//    xTaskCreatePinnedToCore(read_current, "read_current", 2048, &current, 5, NULL, 1);
//    printf("\nread_current task initialized!\n");

	vTaskDelay(500 / portTICK_RATE_MS);
    int cnt = 0;

    printf("\nMain loop initialized!\n");
    while (1)
    {
		cnt++;

		delayMicroseconds(1000000);

		for (int i = 0; i < V_SAMPLES/REASON; i++)
		{
//			vcopy[i] = voltage.samples[i];
			phase_copy[i][0] = phase_a.voltage.samples[i];
			phase_copy[i][1] = phase_a.current.samples[i];
		}

		for (int i = 0; i < V_SAMPLES/REASON; i++)
		{
//			printf("%.2f\n", vcopy[i]);
//			printf("V: %.2f\n", voltage.samples[i]);
			printf("%04.2f %04.2f\n", phase_copy[i][0], phase_copy[i][1]);
			fflush(stdout);
//			printf("A: %.2f\n", current.samples[i]);
			vTaskDelay(10 / portTICK_RATE_MS);
		}

		printf("\n\n");
		fflush(stdout);
    }
}
