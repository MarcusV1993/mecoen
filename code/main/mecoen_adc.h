/*
 * energy_meter_adc.h
 *
 *  Created on: 13 de mar. de 2022
 *      Author: marcus
 */

#ifndef ENRGY_METER_PROJECT_ADC_FUNCTIONS_H_
#define ENRGY_METER_PROJECT_ADC_FUNCTIONS_H_


#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   4          //Multisampling


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


// accessible ADC functions
void init_adc();

void read_phase(void *arg);
// end accessible ADC functions


#endif /* ENRGY_METER_PROJECT_ADC_FUNCTIONS_H_ */
