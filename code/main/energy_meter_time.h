/*
 * energy_meter_time.h
 *
 *  Created on: 13 de mar. de 2022
 *      Author: marcus
 */

#ifndef ENRGY_METER_PROJECT_TIME_RELATED_FUNCTIONS_H_
#define ENRGY_METER_PROJECT_TIME_RELATED_FUNCTIONS_H_

#include "esp_timer.h"
#include "driver/gptimer.h"
#include "esp_system.h"

#define NOP() asm volatile ("nop")


void IRAM_ATTR delayMicroseconds(uint32_t us);

void ojSleepMsec(double miliseconds);


#endif /* ENRGY_METER_PROJECT_TIME_RELATED_FUNCTIONS_H */
