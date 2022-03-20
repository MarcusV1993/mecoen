/*
 * energy_meter_freertos_task_notif.h
 *
 *  Created on: 14 de mar. de 2022
 *      Author: marcus
 */

#ifndef MAIN_ENERGY_METER_FREERTOS_TASK_NOTIF_H_
#define MAIN_ENERGY_METER_FREERTOS_TASK_NOTIF_H_

#include <stdint.h>
#include <stdlib.h>

void StartTransmission( uint8_t *pcData, size_t xDataLength );

void vTransmitEndISR( void );

void vAFunctionCalledFromATask( uint8_t ucDataToTransmit,
                                size_t xDataLength );


#endif /* MAIN_ENERGY_METER_FREERTOS_TASK_NOTIF_H_ */
