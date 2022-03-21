/*
 * energy_meter_time.h
 *
 *  Created on: 13 de mar. de 2022
 *      Author: marcus
 */

#ifndef ENRGY_METER_PROJECT_TIME_RELATED_FUNCTIONS_H_
#define ENRGY_METER_PROJECT_TIME_RELATED_FUNCTIONS_H_

#include <stdint.h>

#ifndef INET6_ADDRSTRLEN
#define INET6_ADDRSTRLEN 48
#endif

#define MECOEN_SNTP_SERVER_NAME "pool.ntp.org"

#define NOP() asm volatile ("nop")

void IRAM_ATTR delayMicroseconds(uint32_t us);

void ojSleepMsec(double miliseconds);

void init_time();

#endif /* ENRGY_METER_PROJECT_TIME_RELATED_FUNCTIONS_H */
