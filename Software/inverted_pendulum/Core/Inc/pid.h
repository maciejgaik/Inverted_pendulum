/*
 * pid.h
 *
 *  Created on: May 17, 2020
 *      Author: Mike
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>
typedef struct {
	int32_t p;
	int32_t i;
	int32_t d;

	int32_t p_max;
	int32_t i_max;
	int32_t d_max;

	int32_t p_min;
	int32_t i_min;
	int32_t d_min;

	int32_t dv;
	int32_t mv;

	int32_t e_last;
	int32_t e_sum;

	int32_t total_max;
	int32_t total_min;

	int32_t control;

	int32_t dt_ms;
} cpid_t;

void pid_init(cpid_t *pid, float p, float i, float d, int32_t dt_ms);

int32_t pid_calc(cpid_t *pid, int32_t mv, int32_t dv);

#endif /* INC_PID_H_ */
