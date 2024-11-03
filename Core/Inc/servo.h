/*
 * servo.h
 *
 *  Created on: 2024年8月19日
 *      Author: 88698
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_
#include "stm32f4xx_hal.h"
#include "mainpp.h"
typedef struct {
    float pos;
    float goalAngle;
    float lastAngle;
    int responseTime;
    bool move;
    bool block_state;
    TIM_HandleTypeDef *htim;
    uint32_t TIM_CHANNEL;

} servo;
void servo_move(servo*servo,float goalAngle,int responseTime);
void servo_run(servo*servo ,int updateFreq);
void blockState();
void servo_setup();
void update_blockstate(bool block_state);

#endif /* INC_SERVO_H_ */
