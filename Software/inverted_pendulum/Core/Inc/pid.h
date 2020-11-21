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
	double p;
	double i;
	double d;

	double p_max;
	double i_max;
	double d_max;

	double p_min;
	double i_min;
	double d_min;

	double dv;
	double mv;

	double e_last;
	double e_sum;

	double total_max;
	double total_min;

	double control;

	double dt_ms;
} cpid_t;

void pid_init(cpid_t *pid, float p, float i, float d, double dt_ms);

double pid_calc(cpid_t *pid, double mv, double dv);

#endif /* INC_PID_H_ */
