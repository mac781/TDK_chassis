/*
 * pathSensor.cpp
 *
 *  Created on: Sep 2, 2024
 *      Author: mac
 */
#include "pathSensor.h"
#include "location.h"
#include "stm32f4xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern float map_x, map_y, last_x, last_y;
extern bool arrive;

#define normal_Speed 1.5
#define w_kp 0.21
#define w_kd 0
#define boundry 1000
#define spin_sp 1.5

uint16_t adcRead[7];
int   check = 0;
float weight_err;
float weight_lasttime = 0;
float weight_change = 0;
float tempSpeed[2];
float path_motor_speed[2];
float path_dis = 0;
/*
adcRead[0]  adc1-7   PA7  right
adcRead[1]  adc1-6   PA6    |
adcRead[2]  adc1-4   PA4    |
adcRead[3]  adc1-1   PA1    V
adcRead[4]  adc1-0   PA0  left
adcRead[5]  adc1-8   PB0  middle right
adcRead[6]  adc1-9   PB1  middle left
*/
void path_setup(){
	if(HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adcRead,7) != HAL_OK)
		check++;
}
void weight(){

	//P's err
	weight_err = (float)(-3*adcRead[0]-adcRead[1]+adcRead[3]+3*adcRead[4])/
						(adcRead[0]+adcRead[1]+adcRead[2]+adcRead[3]+adcRead[4]);

	//D's err
	weight_change = weight_err - weight_lasttime;

	weight_lasttime = weight_err;

	//right
	tempSpeed[0] = normal_Speed + weight_err * w_kp + weight_change * w_kd;
	//left
	tempSpeed[1] = normal_Speed - weight_err * w_kp - weight_change * w_kd;
}
//motor_speed[0]:right motor speed, motor_speed[1]:left motor speed
void path(bool m){

	if(m){
		//turn right
		if(adcRead[5] >= boundry && adcRead[6] < boundry && adcRead[0] < boundry && adcRead[1] < boundry
				&& adcRead[2] < boundry && adcRead[3] < boundry && adcRead[4] < boundry){

			path_motor_speed[0] = spin_sp * -1;
			path_motor_speed[1] = spin_sp;

			while(adcRead[2] < 3 * boundry){}

			ach(1);

			path_motor_speed[0] = 0;
			path_motor_speed[1] = 0;
		}
		//turn left
		else if(adcRead[5] < boundry && adcRead[6] >= 2 * boundry && adcRead[0] < boundry && adcRead[1] < boundry
				&& adcRead[2] < boundry && adcRead[3] < boundry && adcRead[4] < boundry){

			path_motor_speed[0] = spin_sp;
			path_motor_speed[1] = spin_sp * -1;

			while(adcRead[2] < 3 * boundry){}

			ach(1);

			path_motor_speed[0] = 0;
			path_motor_speed[1] = 0;
		}
		//forward
		else{
			for(int j = 0; j < 2; j++){
				for(float i = 2; i >= 0; i -= 0.01){

					if(tempSpeed[j] >= i){
						path_motor_speed[j] = i;
						break;
					}
					else if(tempSpeed[j] > 2){
						path_motor_speed[j] = 2;
						break;
					}
					else if(tempSpeed[j] < 0){
						path_motor_speed[j] = 0;
						break;
					}
				}
			}
		}
	}
	else{
		//forward
		for(int j = 0; j < 2; j++){
			for(float i = 2; i >= 0; i -= 0.01){

				if(tempSpeed[j] >= i){
					path_motor_speed[j] = i;
					break;
				}
				else if(tempSpeed[j] > 2){
					path_motor_speed[j] = 2;
					break;
				}
				else if(tempSpeed[j] < 0){
					path_motor_speed[j] = 0;
					break;
				}
			}
		}
	}
}

//go to (x,y)
void path_moveto(float path_d){

	location_reset(0, 0, 90);

	location_data(0);

	path_dis = path_d;

	ach(0);

	while(!arrive)
		path(1);
}
bool line_check(int type){

	int b = 3000;
	int r_max = 2500, r_min = 500;

	switch(type){

	//front line
	case 1:

		if(adcRead[0] >= b && adcRead[4] >= b)
			return 1;
		else
			return 0;
		break;
	//middle line
	case 2:

		if(adcRead[5] >= b && adcRead[6] >= b)
			return 1;
		else
			return 0;
		break;
	//cross road
	case 3:

		if(adcRead[2] >= b && adcRead[5] >= b && adcRead[6] >= b)
			return 1;
		else
			return 0;
		break;
	//front find line
	case 4:

		if(adcRead[0] >= b || adcRead[1] >= b || adcRead[2] >= b || adcRead[3] >= b || adcRead[4] >= b)
			return 1;
		else
			return 0;
		break;

	//right find line
	case 5:

		if(adcRead[5] >= b)
			return 1;
		else
			return 0;
		break;
	//left find line
	case 6:

		if(adcRead[6] >= b)
			return 1;
		else
			return 0;
		break;
	//spin find line(f)
	case 7:
		if(adcRead[2] >= b)
			return 1;
		else
			return 0;
		break;
	//middle find red
	case 8:
		if(adcRead[5] >= r_min && adcRead[5] <= r_min && adcRead[6] >= r_min && adcRead[6] <= r_min)
			return 1;
		else
			return 0;
		break;
	}
}
