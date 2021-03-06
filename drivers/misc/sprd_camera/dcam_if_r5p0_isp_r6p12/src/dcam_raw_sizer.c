/*
 * Copyright (C) 2017-2018 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/types.h>
#include <linux/vmalloc.h>

#include "cam_sin_cos.h"

#define TOTAL_PHASE          (2 * 4)
#define FILTER_TAP_H         8/* 8->12  xing.huo*/
#define FILTER_TAP_V         4
#define FILTER_TAP_MAX       12
#define FILTER_WINDOW        (1)

#define WEIGHT_BITWIDTH      (10)/* 8->10 xing.huo */
#define WEIGHT_SUM           (1 << WEIGHT_BITWIDTH)
#define WEIGHT_ROUND         (WEIGHT_SUM >> 1)
#define WEIGHT_UPPER         (WEIGHT_SUM - 1)
#define MISMATCH_LIMIT       3
#define PI                   3.14159265357
#define ceil(x)              (((x)-(int)(x)) > 0 ? (int)(x + 1) : (int)(x))
#define fabs(x)              ((x) >= 0 ? (x) : -(x))

#if 0
double cal_linear_weight(double dista, double ratio)
{
	double x = 0;
	double weight = 0;

	x = (dista * ratio * FILTER_WINDOW);
	if ((x < 0) && (x >= -1))
		weight = ((1+x)*ratio);
		/*weight = 0.5 * scale * (1 + x), scale = 1/x_ratio*/
	else if ((x >= 0) && (x <= 1))
		weight = ((1-x)*ratio);
	else
		weight = 0;
	return weight;
}

double cal_cubic_weight(double dista, double ratio)
{
	double y1 = 0;
	double y2 = 0;
	double y3 = 0;
	double weight = 0;
	double a = 0;

	a = -0.5;
	y1 = fabs(dista*ratio*FILTER_WINDOW);
	y2 = y1*y1;
	y3 = y1*y2;

	if ((y1 <= 1))
		weight = (((2 + a) * y3 - (3+a) * y2 + 1) * ratio);
		/*weight = 0.5 * scale * (1 + x), scale = 1/y_ratio*/
	else if ((y1 > 1) && (y1 <= 2))
		weight = ((a * y3 - 5 * a * y2 + 8 * a * y1 - 4 * a) * ratio);
	else
		weight = 0;
	return weight;
}

static double cal_sinc_weight(double dista, double ratio)
{
	double y1 = 0;
	double weight = 0;
	int n = 2;

	y1 = fabs(dista * ratio * FILTER_WINDOW);
	if (y1 == 0)
		weight = 1 * ratio;
	else if (y1 <= n)
		weight = (((0.355768 - 0.487396 * dcam_cos_32(PI * (y1 + n) / n)
			+ 0.144232 * dcam_cos_32(2 * PI * (y1 + n) / n) -
			0.012604 * dcam_cos_32(3 * PI * (y1 + n) / n))
			* (dcam_sin_32(PI * y1) / (PI * y1))) * ratio);
			/*nuttall window*/
	else
		weight = 0;
	return weight;
}

static void normalize_weight(int16_t *norm_weights, double *tmp_weights, int kk)
{
	int weight_max = (1 << WEIGHT_BITWIDTH) - 1;
	double tmp_sum = 0;
	double nor_coef = 0;
	double temp_value = 0;
	uint8_t i = 0;
	int sum = 0;
	int diff = 0;

	for (i = 0; i < kk; i++)
		tmp_sum += tmp_weights[i];

	nor_coef = 1 / tmp_sum;

	for (i = 0; i < kk; i++) {
		temp_value = tmp_weights[i] * nor_coef * WEIGHT_SUM;
		norm_weights[i] = (int16_t)(temp_value);
		sum += norm_weights[i];
	}

	if (norm_weights[kk >> 1] >= weight_max) {
		diff = norm_weights[kk>>1] - weight_max;
		norm_weights[kk >> 1] = weight_max;
		norm_weights[(kk >> 1) - 1] += diff;
	}

	if (norm_weights[(kk >> 1) - 1] >= weight_max) {
		diff = norm_weights[(kk >> 1) - 1] - weight_max;
		norm_weights[(kk >> 1) - 1] = weight_max;
		norm_weights[kk >> 1] += diff;
	}

	/* norm_weights cannot be smaller than weight_min*/

#if 0
	/* coefficients check*/
	for (int j = 1; j < kk - 1; ++j) {
		SCI_ASSERT(norm_weights[j] <= weight_max);
		SCI_ASSERT(norm_weights[j] >= weight_min);
	}
#endif
	if (sum != WEIGHT_SUM) {
		diff = WEIGHT_SUM - sum;
		if (diff > 0)
			norm_weights[kk >> 1] = norm_weights[kk >> 1] + diff;
		else
			norm_weights[0] = norm_weights[0] + diff;
	}
}

#define CLIP(input, top, bottom) \
	{ if (input > top) \
		input = top;\
	if (input < bottom) \
		input = bottom; }

static void cal_scaler_weight(uint16_t src_size, uint16_t dst_size,
		uint8_t hor_or_ver, int16_t *filter_weight,
		uint8_t *filter_phase, uint8_t *filter_tap)
{
	double scale = 0;
	int N = TOTAL_PHASE;
	int kw = 0, tap = 0;
	int i, index = 0, phase = 0, idx_lb = 0, idx_ub = 0;
	double weight_phase[FILTER_TAP_MAX] = {0};
	double (*weight_func)(double, double) = NULL;/* function pointer*/

	scale = (dst_size * 1.0) / src_size;
	/*assert( scale >= 0.5 && scale <= 1.0 );*/

	if (hor_or_ver) {
		kw = 4;/*cubic or windowed sinc*/
		tap = FILTER_TAP_H;
		/*weight_func = cal_cubic_weight;*/
		weight_func = cal_sinc_weight;
	} else {
		kw = 2;/*linear*/
		tap = FILTER_TAP_V;
		/*weight_func = cal_linear_weight;*/
		weight_func = cal_sinc_weight;
	}

	idx_ub = (int)ceil(kw * TOTAL_PHASE / (2 * scale));
	idx_lb = -idx_ub;

	for (phase = 0; phase < N; phase++) {
		int offset = -tap / 2 + 1;

		for (i = 0; i < tap; i++) {
			index = N * (i + offset) - phase;
			CLIP(index, idx_ub, idx_lb);
			weight_phase[i] = weight_func(index * 1.0 / N, scale);
		}
		normalize_weight(filter_weight, weight_phase, tap);
		filter_weight += tap;
	}

	*filter_phase = N;
	*filter_tap = tap;
}

#define coeff_v(coeff, i, j)   (*(coeff + FILTER_TAP_V * i + j) & 0x7FF)
#define coeff_h(coeff, i, j)   (*(coeff + FILTER_TAP_H * i + j) & 0x7FF)
#endif


int gen_rawsizer_coeff(uint16_t src_width, uint16_t src_height,
		uint16_t dst_width, uint16_t dst_height, uint32_t *coeff_buf)
{
#if 0
	int16_t *hor_weight_table;
	int16_t *ver_weight_table;
	int i = 0;
	uint8_t hor_N = 0, hor_tap = 0;
	uint8_t ver_N = 0, ver_tap = 0;

	hor_weight_table = vzalloc(TOTAL_PHASE * FILTER_TAP_H
			* sizeof(int16_t));
	ver_weight_table = vzalloc(TOTAL_PHASE * FILTER_TAP_V
			* sizeof(int16_t));

	if (!coeff_buf) {
		return -1;
	};

	if (!hor_weight_table || !ver_weight_table) {
		vfree(hor_weight_table);
		vfree(ver_weight_table);
		return -1;
	}

	cal_scaler_weight(src_width, dst_width, 1,
		hor_weight_table, &hor_N, &hor_tap);
	cal_scaler_weight(src_height, dst_height, 0,
		ver_weight_table, &ver_N, &ver_tap);

	for (i = 0; i < (TOTAL_PHASE); i++) {
		*coeff_buf++ = ((uint32_t)coeff_v(ver_weight_table, i, 0) << 16)
			+ coeff_v(ver_weight_table, i, 1);
		*coeff_buf++ = ((uint32_t)coeff_v(ver_weight_table, i, 2) << 16)
			+ coeff_v(ver_weight_table, i, 3);
	}
	for (i = 0; i < (TOTAL_PHASE); i++) {
		*coeff_buf++ = ((uint32_t)coeff_h(hor_weight_table, i, 0) << 16)
			+ coeff_h(hor_weight_table, i, 1);
		*coeff_buf++ = ((uint32_t)coeff_h(hor_weight_table, i, 2) << 16)
			+ coeff_h(hor_weight_table, i, 3);
		*coeff_buf++ = ((uint32_t)coeff_h(hor_weight_table, i, 4) << 16)
			+ coeff_h(hor_weight_table, i, 5);
		*coeff_buf++ = ((uint32_t)coeff_h(hor_weight_table, i, 6) << 16)
			+ coeff_h(hor_weight_table, i, 7);
	}
	vfree(hor_weight_table);
	vfree(ver_weight_table);
#endif
	return 0;
}
