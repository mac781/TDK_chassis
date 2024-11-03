/*
 * servo.cpp
 *
 *  Created on: 2024年8月19日
 *      Author: 88698
 */
#include "servo.h"
#include "mainpp.h"
extern TIM_HandleTypeDef htim12;
int goalAngle;
int responseTime = 500;
int servoAngle[2] = {50,160};
//F front, B back, R right, M middle, L left
servo servoFL = {0, 0, 0, 1000, true, false, &htim12, TIM_CHANNEL_2};
servo servoBL = {0, 0, 0, 1000, true, false, &htim12, TIM_CHANNEL_1};

//servo servo[0]{0,0,0,2000,true, &htim3, TIM_CHANNEL_1 };
void servo_setup(){
	HAL_TIM_PWM_Start(servoFL.htim, servoFL.TIM_CHANNEL);
	HAL_TIM_PWM_Start(servoBL.htim, servoBL.TIM_CHANNEL);
}
void servo_move(servo*servo,float goalAngle,int responseTime){
	servo -> goalAngle = goalAngle;
	servo -> responseTime = responseTime;
	servo -> move = true;
}
void servo_run(servo*servo ,int updateFreq){//updateFreq = timer interrupt frequency Hz
	if (servo -> move == true){
		if ((int)servo -> pos == (int)servo -> goalAngle){
    	servo -> move = false;
    	servo -> lastAngle = servo -> goalAngle;
        }else{
         float distance = servo -> goalAngle - servo -> lastAngle;
         servo -> pos += distance/(servo -> responseTime * updateFreq / 1000);
         __HAL_TIM_SET_COMPARE(servo -> htim, servo -> TIM_CHANNEL,600+10*(int)servo -> pos);
         //t++;
        }
	}
}
//for script

void blockState(){//控制servo開關，true open ,false close
	//寫劇本改bloack_state_FL和bloack_state_BL兩個變數就好
	servo_run(&servoFL, 1000);
	servo_run(&servoBL, 1000);
	//
	if (servoFL.block_state == true){//左前servo
		servo_move(&servoFL, servoAngle[0],1000);
	}else{
		servo_move(&servoFL, servoAngle[1],1000);
	}
	if (servoBL.block_state == true){//左後servo
		servo_move(&servoBL, servoAngle[1],1000);
	}else{
		servo_move(&servoBL, servoAngle[0],1000);
		}
}
void update_blockstate(bool block_state){
	//block_state為true時擋板打開,false擋板關閉
	servoFL.block_state = block_state;
	servoBL.block_state = block_state;
	wait(responseTime + 500);

}
