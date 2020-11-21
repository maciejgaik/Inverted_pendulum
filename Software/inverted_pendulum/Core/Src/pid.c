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

	pid->p_max = INT32_MAX;
	pid->p_min = INT32_MIN;

	pid->i_max = INT32_MAX;
	pid->i_min = INT32_MIN;

	pid->d_max = INT32_MAX;
	pid->d_min = INT32_MIN;

	pid->e_last = 0;
	pid->e_sum = 0;

	pid->total_max = INT32_MAX;
	pid->total_min = INT32_MIN;

	pid->dt_ms = dt_ms;
}

double pid_calc(cpid_t *pid, double mv, double dv) {

	double p, i, d, e, total;
	pid->mv = mv;
	pid->dv = dv;

//WYLICZANIE BLEDU
	e = dv - mv;
	//if(e > -2 && e < 2) return 0;

//CZLON PROPORCJONALNY
	p = pid->p * e;
	if (p > pid->p_max)
		p = pid->p_max;
	else if (p < pid->p_min)
		p = pid->p_min;

//CZLON CALKUJACY
	i=pid->e_sum;
	i += pid->i * pid->dt_ms * e / 1000;
	if (i > pid->i_max)
		i = pid->i_max;
	else if (i < pid->i_min)
		i = pid->i_min;
	pid->e_sum=i;

//CZLON ROZNICZKUJACEGO
	d = 1000 * pid->d * (e - pid->e_last) / pid->dt_ms;
	if (d > pid->d_max)
		d = pid->d_max;
	else if (d < pid->d_min)
		d = pid->d_min;

//Sumowanie
	total = p + i + d;
	if (total > pid->total_max)
		total = pid->total_max;
	else if (total < pid->total_min)
		total = pid->total_min;

	pid->control = total;
	pid->e_last = e;
	return pid->control;
}

