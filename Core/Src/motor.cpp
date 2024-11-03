/*
 * motor.cpp
 *
 *  Created on: Sep 2, 2024
 *      Author: mac
 */
#include "motor.h"
#include "string.h"
#include "math.h"
#include "location.h"
#include "pathSensor.h"

int16_t enc_a,enc_b,enc_c,enc_d;
const int resolution = 512;
const int reduction_ratio = 64;
float ki_wheel = 273.2793;
float kp_wheel= 7.77902;
float ki_cascade = 273.2793;
float kp_cascade= 7.77902;
float sp[4] ={0};
float cascade_speed = 2;
int arr = 4199;
float span = 0.001;
float v1 = -1, v2;
int timer = 0 ;
int cascade = 0;
//float p = 3.5, i= 10.1,turns;
int a[2] = {0};
float goalHeight = 9.2;
#define epsilon 0.01

float GO_BACK = 1;
int cascade_state = 0;// 0 = 到下面,1 = 運作中,2 = 到上面
bool cascade_arrive_down[2] = {1, 1};//到到下面時微動觸發微動觸發

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;

extern float chassis_right_wheel_angspeed, chassis_left_wheel_angspeed;

PID_controller PID_controllers[4] = {
    {kp_cascade, ki_cascade, 0, span, 0, arr,
    		GPIOC, GPIO_PIN_12, &htim1, TIM_CHANNEL_1},  // cascade_motor_a
    {kp_cascade, ki_cascade, 0, span, 0, arr,
		    GPIOC, GPIO_PIN_13, &htim1, TIM_CHANNEL_2},// cascade_motor_a
    {kp_wheel, ki_wheel, 0, span, 0, arr,
    		GPIOB, GPIO_PIN_12, &htim1, TIM_CHANNEL_4},  // wheel_motor_left
    {kp_wheel, ki_wheel,0, span, 0, arr,
        	GPIOB, GPIO_PIN_13, &htim1, TIM_CHANNEL_3}   // wheel_motor_right
};
DC_motor DC_Motors[4] = {
	{&PID_controllers[0],&htim2,0,0,0,reduction_ratio},
	{&PID_controllers[1],&htim4,0,0,0,reduction_ratio},
	{&PID_controllers[2],&htim8,0,0,0,reduction_ratio},
	{&PID_controllers[3],&htim3,0,0,0,reduction_ratio}
};

void DCmotor_setup(){
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);//motor[1]
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);//motor[2]
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);//motor[3]
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_2);//motor[4]
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void PI_control_run(DC_motor *motor,float sp) {
    float error, u_a = 0;
    int pul = 0;
    float bound = 1 / motor->PID_Controllers->ki;
    motor->PID_Controllers->setpoint = sp;
    error = motor->PID_Controllers->setpoint - motor->speed;
    motor->PID_Controllers->integral += error * motor->PID_Controllers->span;

    if (motor->PID_Controllers->integral
    		> bound) motor->PID_Controllers->integral = bound;
    else if (motor->PID_Controllers->integral
    		< -bound) motor->PID_Controllers->integral = -bound;
    u_a = motor->PID_Controllers->kp * error
    		+ motor->PID_Controllers->ki * motor->PID_Controllers->integral;
    if (u_a > 1) u_a = 1;
    else if (u_a < -1) u_a = -1;

    if (u_a > 0) {
        pul = (int)(u_a * motor->PID_Controllers->arr);
        HAL_GPIO_WritePin(motor->PID_Controllers->gpioPort, motor->PID_Controllers->gpioPin, GPIO_PIN_SET);
    } else if (u_a < 0) {
        pul = (int)(-u_a * motor->PID_Controllers->arr);
        HAL_GPIO_WritePin(motor->PID_Controllers->gpioPort, motor->PID_Controllers->gpioPin, GPIO_PIN_RESET);
    } else {
        pul = 0;
    }
    __HAL_TIM_SET_COMPARE(motor->PID_Controllers->htim, motor->PID_Controllers->TIM_CHANNEL, pul);
}

void getState(DC_motor *motor,int sign_speed,int sign_height){
	int16_t enc ;
	enc = __HAL_TIM_GetCounter(motor->htim);
	motor->speed = sign_speed*(float)enc /(4*resolution*motor->reduction_ratio*span);
	__HAL_TIM_SetCounter(motor->htim,0);
	motor->currentHeight += motor->speed*span*sign_height;
}
//void set_goalHeight(DC_motor *cascade,float height_setpoint,float *velocity_sp,float speed){
//	if (abs(cascade->currentHeight - height_setpoint) >= epsilon){
//		if(height_setpoint >= cascade->currentHeight){
//			*velocity_sp = -speed;
//		}else if(height_setpoint <= cascade->currentHeight){
//			*velocity_sp = speed;
//		}
//	}else{
//		*velocity_sp = 0;
//	}
//}
int k = 0;
void Cascade_run(){//put in timer_IT
	//改GO_BACK就好,false 上升,true 下降
	if(GO_BACK == 0){//上升
		//先修改底下微動到點判斷
		cascade_arrive_down[0] = false;
		cascade_arrive_down[1] = false;
		//給速度
		if(DC_Motors[0].currentHeight < goalHeight){
			sp[0] = cascade_speed;
		}else if(DC_Motors[0].currentHeight >= goalHeight)
			sp[0] = 0;
		if(DC_Motors[1].currentHeight < goalHeight){
			sp[1] = cascade_speed;
		}else if(DC_Motors[1].currentHeight >= goalHeight)
			sp[1] = 0;
		//到點判斷
		if(DC_Motors[0].currentHeight >= goalHeight &&
				DC_Motors[1].currentHeight >= goalHeight){//到達上面
			cascade_state = 2;
		}else{
			cascade_state = 1;
		}
	}
	else if(GO_BACK == 1){//下降
		//給速度
		if (cascade_arrive_down[0] == false){
			sp[0] = -cascade_speed;
		}else if (cascade_arrive_down[0] == true||DC_Motors[0].currentHeight <= -0.5){
			sp[0] = 0;
		}
		if (cascade_arrive_down[1] == false){
			sp[1] = -cascade_speed;
		}else if (cascade_arrive_down[1] == true||DC_Motors[1].currentHeight <= -0.5){
			sp[1] = 0;
		}
		//到點判斷
		if(cascade_arrive_down[0] == true && cascade_arrive_down[1] == true){
			cascade_state = 0;
		}else{
			cascade_state = 1;
		}

	}
	else if(GO_BACK == 2){
		cascade_state = 1;
			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_RESET){
				sp[1] = -cascade_speed;
			}
			else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET){
				sp[1] = 0;
			}
			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_RESET){
				sp[0] = -cascade_speed;
					}
			else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET){
				sp[0] = 0;
					}
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET &&
					HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET){
				cascade_state = 0;
			}
		}
}

//put in EXTI
void updateLocation(uint16_t GPIO_Pin){//get ball location = 0,put ball location = 1
	  switch (GPIO_Pin) {
	  case GPIO_PIN_9://right
		  a[0]++;
		  if (HAL_GPIO_ReadPin(GPIOC, GPIO_Pin) == GPIO_PIN_SET){
		  DC_Motors[0].currentHeight = 0;
		  if(GO_BACK == true){
		  cascade_arrive_down[0] = true;
		  sp[0] = 0;
		  }
		  }
		  ;
		  break;
	  case GPIO_PIN_8://left
		  a[1]++;
		  if (HAL_GPIO_ReadPin(GPIOC, GPIO_Pin) == GPIO_PIN_SET){
	      DC_Motors[1].currentHeight = 0;
	      if(GO_BACK == true){
	      cascade_arrive_down[1] = true;
	      sp[1] = 0;
	      }
		  }
	  	  break;
	  default:
		  break;
	  }
}
void DCmotor_run(){
	//get speed from encoder, and calculate height
	getState(&DC_Motors[0],1,1);//
	getState(&DC_Motors[1],-1,-1);//
	getState(&DC_Motors[2],1,1);//R_wheel
	getState(&DC_Motors[3],-1,-1);//L_wheel
	//set goal height of cascade
//	set_goalHeight(&DC_Motors[0],goalHeight[0],&sp[0],cascade_speed);
//	set_goalHeight(&DC_Motors[1],goalHeight[1],&sp[1],cascade_speed);
    // PI control DCmotor with velocity set point
	PI_control_run(&DC_Motors[0],sp[0]);
	PI_control_run(&DC_Motors[1],-sp[1]);
	PI_control_run(&DC_Motors[2],sp[2]);//sp[2]
	PI_control_run(&DC_Motors[3],sp[3]);//sp[3]

	//motor_run++;
	}

//read encoder
void encodersp(float *encsp){

	encsp[1] = -1 * DC_Motors[3].speed;
	encsp[0] = DC_Motors[2].speed;
}

//adjust speed, 0 path, 1 integral, 2 stop
void speedOutput(int m){

	speed_change(m);
	location_data(m);
	weight();

	sp[2] = chassis_right_wheel_angspeed;//給正速度往前
	sp[3] = -1 * chassis_left_wheel_angspeed;//給負速度往前
}
bool reach_destination(){
	if (GO_BACK  == 1 && cascade_state == 0){
		return 1;}
	else if (GO_BACK  == 2 && cascade_state == 0){
		k++;
		return 1;}
	else if (GO_BACK  == false && cascade_state == 2)
		return 1;
	else{
		return 0;
	}
}
void update_cascadeState(float goback){//goback = true 時下降，false上升
	GO_BACK = goback;
	while(!reach_destination()){
	}
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*htim){
//	if (htim -> Instance == TIM6){
//
//
//	}
//}
