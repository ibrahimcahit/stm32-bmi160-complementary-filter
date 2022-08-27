/*
 * bmi160_wrapper.h
 *
 *  Created on: Mar 30, 2022
 *      Author: Ibrahim Ozdemir
 *      GitHub: ibrahimcahit
 */

#ifndef BMI160_WRAPPER_H_
#define BMI160_WRAPPER_H_

#include "bmi160.h"
#include "common_porting.h"
#include "bmi160_defs.h"

#define 	TRUE 			1
#define 	FALSE 			0
#define 	PRINTIF_DEBUG 		0

typedef struct {

	float BMI160_Ax_f32, BMI160_Ay_f32, BMI160_Az_f32;
	float BMI160_Gx_f32, BMI160_Gy_f32, BMI160_Gz_f32;

	int8_t INIT_OK_i8;
} BMI160_t;

int8_t BMI160_init();
int8_t bmi160ReadAccelGyro(BMI160_t *DataStruct);
int8_t start_foc();

void set_bmi160_Ares();
void set_bmi160_Gres();
void get_bmi160_Ares();
void get_bmi160_Gres();

#endif /* BMI160_WRAPPER_H_ */
