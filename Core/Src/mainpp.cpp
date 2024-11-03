/*
 * mainpp.cpp
 *
 *  Created on: Sep 2, 2024
 *      Author: mac
 */
#include "mainpp.h"
#include "motor.h"
#include "communication.h"
#include "location.h"
#include "pathSensor.h"
#include "servo.h"
#include "string.h"
#include "stm32f4xx_hal.h"

#define epsilon 0.1
#define craw_to_middle 0

int delay_count = 0,divide_10_count = 0,divide_100_count;

extern bool arrive;
extern bool nextPoint;
extern float statusData[4];
extern servo servoFL;
extern servo servoBL;

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim13;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart4;

int move_mode = 2;

void setup(){
	//UART_setup();

	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim13);
	DCmotor_setup();
	path_setup();
	UART_setup();
	servo_setup();
	arrive = 0;
}
int divide(int times,int *count){

	if (*count > times){
		*count = 0;
		return 1;

	}else{
		return 0;
	}
}

void wait(int time){//time單位為ms

	HAL_TIM_Base_Start_IT(&htim13);

	while(delay_count < time){
	}

	HAL_TIM_Base_Stop_IT(&htim13);

	delay_count = 0;
}

void main_function(){

	while(1){

		if(nextPoint){

			arrive = 0;

			//path
			if(statusData[0] == 1){

				move_mode = 0;
				path_moveto(statusData[2]);
			}
			//integral
			else if(statusData[0] == 2){

				move_mode = 1;
				integral_moveto(statusData[1], statusData[2], statusData[3]);
			}
			//stop
			else if(statusData[0] == 3){

				move_mode = 2;
				arrive = 1;
			}
			//find line
			else if(statusData[0] == 4){

				move_mode = 3;
				move_to_line(statusData[1], statusData[2]);
			}
			//reset
			else if(statusData[0] == 5){

				move_mode = 2;
				location_reset(statusData[1], statusData[2], statusData[3]);
			}
			//cascade
			else if(statusData[0] == 6){

				move_mode = 2;

				if(statusData[1] == 1)
					update_cascadeState(1);

				else if(statusData[1] == 0)
					update_cascadeState(0);

				else if(statusData[1] == 2)
					update_cascadeState(2);

				arrive = 1;
			}
			//cascade servo
			else if(statusData[0] == 7){

				move_mode = 2;

				if(statusData[1])

					update_blockstate(1);
				else
					update_blockstate(0);

				arrive = 1;
			}
			nextPoint = 0;
			move_mode = 2;
		}
	}
}
int tt,ttt;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*htim){

	if (htim -> Instance == TIM7){
		divide_10_count++;
		divide_100_count++;
		speedOutput(move_mode);
		DCmotor_run();
		Cascade_run();
		blockState();

		if(divide(10,&divide_10_count)){

			}
		if(divide(100,&divide_100_count)){
			tt++;
			UART_TRANSMIT();
		}
		//transmit_test();
		//HAL_UART_Transmit_DMA(&huart1, buffer_TX, sizeof(buffer_TX));
		//HAL_UART_Transmit_IT(&huart1, buffer_TX, sizeof(buffer_TX));
	}
	 if (htim -> Instance == TIM13){
		// UART_TRANSMIT();
		 delay_count++;
		 //UART_ReceiveData_Status_IT(&huart4, &statusData[0], &statusData[1], &statusData[2], &statusData[3]);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

//UART test
	updateLocation(GPIO_Pin);

//		UART_TRANSMIT();
////
//
}
