/*
 * pid.c
 *
 *  Created on: May 17, 2020
 *      Author: Mike
 */
#include "pid.h"

void pid_init(cpid_t *pid, float p, float i, float d, double dt_ms) {
	pid->p = p;
	pid->i = i;
	pid->d = d;

	pid->e_last = 0;
	pid->e_sum = 0;

	pid->total_max = INT32_MAX;
	pid->total_min = INT32_MIN;

	pid->dt_ms = dt_ms/1000;
}

double pid_calc(cpid_t *pid, double mv, double dv) {

	double p, i, d, e, total;

//WYLICZANIE BLEDU
	e = dv - mv;

//CZLON PROPORCJONALNY
	p = pid->p * e;

//CZLON CALKUJACY
	i = pid->e_sum;
	i += (pid->i * pid->dt_ms * e);

//CZLON ROZNICZKUJACEGO
	d = pid->d * (e - pid->e_last) / pid->dt_ms;

//SUMOWANIE
	total = p + i + d;
	if (total > pid->total_max)
		total = pid->total_max;
	else if (total < pid->total_min)
		total = pid->total_min;

	pid->control = total;
	pid->e_last = e;
	pid->e_sum = i;
	return pid->control;
}

