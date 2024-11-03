/*
 * motor.h
 *
 *  Created on: Sep 2, 2024
 *      Author: mac
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32f4xx_hal.h"
#include "mainpp.h"

typedef struct {
    float kp;
    float ki;
    float integral;
    float span;
    float setpoint;
    int arr;
    GPIO_TypeDef *gpioPort;
    uint16_t gpioPin;
    TIM_HandleTypeDef *htim;
    uint32_t TIM_CHANNEL;
} PID_controller;

typedef struct {
	PID_controller *PID_Controllers;
	TIM_HandleTypeDef *htim;//encoder_timer
	float speed;
	float currentHeight;
	float goalHeight;
	float reduction_ratio;
} DC_motor;

void DCmotor_setup();
void getState(DC_motor *lifter,int sign);
void PI_control_run(DC_motor *motor,float sp) ;
void DCmotor_run();
void set_goalHeight(DC_motor *cascade,float height_setpoint,float *velocity_sp,float speed);
void encodersp(float *encsp);
void speedOutput(int m);
void Cascade_run();
void updateLocation(uint16_t GPIO_Pin);
void update_cascadeState(float goback);

#endif /* INC_MOTOR_H_ */
