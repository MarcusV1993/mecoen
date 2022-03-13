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

#endif /* MAIN_DEFINITIONS_H_ */
