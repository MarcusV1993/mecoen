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


/**********************************************************************/
// definitions
//// definitions common
#define N_ARRAY_LENGTH 2048
#define REASON 4
//// end definitions common


//// definitions time
#define NOP() asm volatile ("nop")

////// definitions time ntp
#ifndef INET6_ADDRSTRLEN
#define INET6_ADDRSTRLEN 48
#endif

#define MECOEN_SNTP_SERVER_NAME "pool.ntp.org"
////// end definitions time ntp
//// end definitions time


//// definitions ADC
////// definitions ADC peripheral reference voltage
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
////// end definitions ADC peripheral reference voltage

////// definitions ADC sampling
#define SAMPLING_FREQUENCY 2500
#define NO_OF_SAMPLES   8          //Multisampling
////// end definitions ADC sampling

////// definitions ADC voltage sensor parameters
#define ZMPT101B_VCC            3.3
#define ZMPT101B_R1			   9840 // 10k
#define ZMPT101B_R2			   9970 // 10k

#define ZMPT101B_VMAX		 0.8202 // Calibrar
////// end definitions ADC voltage sensor parameters

////// definitions ADC current sensor parameters
#define SCT013_VCC 				3.3
#define SCT013_NUMBER_TURNS    2000
#define SCT013_BURDEN_RESISTOR  466 // 470
#define SCT013_R1             21700 // 22k
#define SCT013_R2              9870 // 10k
////// end definitions ADC current sensor parameters

////// definitions ADC Vdc equivalent 0 V or 0 A
#define ZMPT101B_VDC 419
#define   SCT013_VDC 504
////// end definitions ADC Vdc equivalent 0 V or 0 A

////// definitions ADC conversion rates to real world values
#define ZMPT101B_CONSTANT_MULTIPLIER 0.60595
#define SCT013_CONSTANT_MULTIPLIER 0.00932
////// end definitions ADC conversion rates to real world values
//// end definitions ADC


//// definitions wi-fi
////// definitions wi-fi AP default configuration
#define MECOEN_WIFI_AP_SSID      "mecoen"
#define MECOEN_WIFI_AP_PASS      "12345678"
#define MECOEN_WIFI_AP_CHANNEL   9 // 1 - 13
#define MECOEN_WIFI_AP_MAX_STA_CONN       5
////// end definitions wi-fi AP default configuration

////// definitions wi-fi STA default configuration
// To-do: Configure for UTF characters
#define MECOEN_WIFI_STA_SSID      "Goncalves_2g" /*"POCO X3 Pro"*/
#define MECOEN_WIFI_STA_PASS      "3NMLWWXFFW"
#define MECOEN_WIFI_MAXIMUM_RETRY  5

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
////// end definitions wi-fi STA default configuration
//// end definitions wi-fi


//// definitions storage
////// definitions storage time
#define STORAGE_PERIOD 5 // minutes
#define STORAGE_MAINTAIN 63 // days
////// end definitions storage time
//// end definitions storage
// end definitions


/**********************************************************************/
// consts
//// consts common
const int N_ARRAY_LENGTH2 = 2 * N_ARRAY_LENGTH;
//// end consts common


//// consts ADC
////// consts ADC sampling
const int SAMPLING_FREQUENCY2 = 2 * SAMPLING_FREQUENCY;
////// end consts ADC sampling
//// end consts ADC


//// consts storage
static const int storage_period_s = STORAGE_PERIOD * 60;
//// end consts storage
// end consts


/**********************************************************************/
// structures
//// structure to store magnitude and phase of a signal
typedef struct Mag_phase
{
	float mag;
	float phase_max_mag;
} Mag_phase;
//// end structure to store magnitude and phase of a signal


//// structure to store signal readings, as well as it's Root Mean Squared value, and the array for FFT calculations
typedef struct Signal
{
	float samples[N_ARRAY_LENGTH];
	float rms_previous = 0, rms = 0;

	// FFT
	// working complex array
	__attribute__((aligned(16)))
	float y_cf[N_ARRAY_LENGTH2];
	// Pointers to result arrays
	float *y1_cf = &y_cf[0];
	float *y2_cf = &y_cf[N_ARRAY_LENGTH];
	Mag_phase mag_phase;
	// Sum of y1 and y2
//	__attribute__((aligned(16)))
//	float sum_y[SAMPLING_FREQUENCY/2];
} Signal;
//// end structure to store signal readings, as well as it's Root Mean Squared value, and the array for FFT calculations


//// structure for voltage and current readings in a phase of the circuit, and phase power calculation results
typedef struct Circuit_phase
{
	Signal voltage, current, power;
	float power_apparent, power_active, power_reactive, power_factor, freq;
} Circuit_phase;
//// end structure for voltage and current readings in a phase of the circuit, and phase power calculation results
// end structures



/**********************************************************************/
// ADC variables
//// to save moving average
extern float zmpt101b_vdc;
extern float  sct013_vdc;
////end to save moving average
// end ADC variables


// Task sincronization variables
//extern TaskHandle_t task_fft, task_adc;
//const UBaseType_t indexToWaitOn = 1;
extern SemaphoreHandle_t semaphore_adc_dc, semaphore_adc, semaphore_fft;
// end Task sincronization variables

#endif /* MAIN_DEFINITIONS_H_ */
