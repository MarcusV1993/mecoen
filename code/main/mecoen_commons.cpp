/*
 * mecoen_commons.cpp
 *
 *  Created on: 17 de mar. de 2022
 *      Author: marcus
 */

#include "mecoen_commons.h"
#include "mecoen_definitions.h"

void
init_phase(Circuit_phase *phase)
{
	phase->voltage.y1_cf = &phase->voltage.y_cf[0];
	phase->voltage.y2_cf = &phase->voltage.y_cf[SAMPLING_FREQUENCY];
	phase->current.y1_cf = &phase->current.y_cf[0];
	phase->current.y2_cf = &phase->current.y_cf[SAMPLING_FREQUENCY];
	phase->power.y1_cf = &phase->power.y_cf[0];
	phase->power.y2_cf = &phase->power.y_cf[SAMPLING_FREQUENCY];
}
