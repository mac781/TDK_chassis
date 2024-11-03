/*
 * mainpp.h
 *
 *  Created on: Sep 2, 2024
 *      Author: mac
 */

#ifndef INC_MAINPP_H_
#define INC_MAINPP_H_

#include <stdbool.h>

#ifdef __cplusplus
extern"C"{
#endif

	#include "stm32f4xx_hal.h"
	void main_function();
	void setup();
	void wait(int time);
	int divide(int times,int *count);

#ifdef __cplusplus
}
#endif

#endif /* INC_MAINPP_H_ */
