/*
 * location.cpp
 *
 *  Created on: Sep 2, 2024
 *      Author: mac
 *
         front
           90
           y
            ︿
           |
           |
           |
 180--------------->x 0
           |
           |
           |
          -90
 */
#include "location.h"
#include "motor.h"
#include "pathSensor.h"
#include "stdlib.h"
#include "cmath"

#define pi 3.14159

extern float path_dis, path_motor_speed[2];
extern uint16_t adcRead[7];
extern int move_mode;

//length:cm，time:s
float wheel_radius = 10;
float chassis_radius = 31.86625;
float timer_span = 0.001;

//wheel angular speed
float chassis_right_wheel_angspeed;
float chassis_left_wheel_angspeed;

//chassis speed
float chassis_speed;
float chassis_angspeed;

//encoder read
float encRead[2];

//map info
float map_x = 0;
float map_y = 0;
float map_theta_front = pi / 2;
float map_theta_back = -1 * map_theta_front;

//target info
float goal_x = 0;
float goal_y = 0;
float to_theta = 0;
float goal_theta = 0;

float start_x = 0;
float start_y = 0;
float start_theta_front = 0;
float start_theta_back = 0;
float end_theta_front = 0;

//last distance info
float last_x = 0;
float last_y = 0;
float last_theta = 0;
float last_goal_theta = 0;

float SP = 0, spin = 0;

float max_sp = 60;
float max_spin = 0.8;
float min_sp = 5;
float min_spin = 0.05;
float Correction_sp = max_sp * 0.9;

int mode = 0;
int check_sf = 0;
int check_sb = 0;
int check_f = 0;
int check_b = 0;
int check_a = 0;
int check_e = 0;

bool line_find = 0;
bool arrive = 0;

void cis_speedTransfer_modle(){

	//rps
	encodersp(encRead);

	// 0:right, 1:left
	//cm/s
	chassis_speed 	 = ((encRead[0] + encRead[1]) / 2) * (2 * pi * wheel_radius) / 2;

	//rps
	chassis_angspeed = ((encRead[0] - encRead[1]) / 2) * wheel_radius / chassis_radius / 2;
}
//input cm/s, rps
void trans_speedTransfer_modle(float chassis_sp, float chassis_angsp){

	//rps
	chassis_right_wheel_angspeed = (chassis_sp / (2 * pi) + chassis_radius * chassis_angsp) / wheel_radius * 2;
	chassis_left_wheel_angspeed  = (chassis_sp / (2 * pi) - chassis_radius * chassis_angsp) / wheel_radius * 2;
}
void location_reset(float x, float y, float ang){

	map_x = x;
	map_y = y;
	map_theta_front = ang * pi / 180;
	map_theta_back = -1 * map_theta_front;

	goal_x = 0;
	goal_y = 0;
	to_theta = 0;
	goal_theta = 0;

	start_x = 0;
	start_y = 0;
	start_theta_front = 0;
	start_theta_back = 0;
	end_theta_front = 0;

	last_x = 0;
	last_y = 0;
	last_theta = 0;
	last_goal_theta = 0;

	chassis_right_wheel_angspeed = 0;
	chassis_left_wheel_angspeed = 0;

	arrive = 1;
}
//update location info 0 path, 1 integral, 2 stop
void location_data(int MODE){

	cis_speedTransfer_modle();

	//integral location
	if(MODE == 1){
		last_x = goal_x - map_x;
		last_y = goal_y - map_y;

		to_theta = std::atan2(last_y, last_x);

		last_theta = std::fmin(std::abs(to_theta - map_theta_front), std::abs(to_theta - map_theta_back));

		if(std::abs(goal_theta - map_theta_front) < pi / 2)
			last_goal_theta = std::abs(goal_theta - map_theta_front);
		else
			last_goal_theta = 2 * pi - std::abs(goal_theta - map_theta_front);

		map_x += chassis_speed * timer_span * std::cos(map_theta_front);
		map_y += chassis_speed * timer_span * std::sin(map_theta_front);

		//record orientation change(rad)
		map_theta_front += chassis_angspeed * timer_span * 2 * pi;

		if(map_theta_front > 0)
			map_theta_back = map_theta_front - pi;
		else
			map_theta_back = map_theta_front + pi;

		//rad:+pi ~ -pi
		if(map_theta_front > std::atan2(0, -1) || map_theta_front <= -1 * std::atan2(0, -1))
			map_theta_front *= -1;
	}
	//path location
	else if(MODE == 0){

		last_y = path_dis - map_y;

		if(path_dis)
			map_y += chassis_speed * timer_span;

		if(last_y <= 0.1 && path_dis != 0)
			arrive = 1;
	}

}
//goto (x,y), orientation 180 ~ -180 degree，input 1000 to not spin
void integral_moveto(float x, float y, float orientation){

	bool direction = 1;
	bool ar = 0;
	bool Correction = 0;

	arrive = 0;
	mode = 0;

	SP = 0;
	spin = 0;

	start_x = map_x;
	start_y = map_y;
	start_theta_front = map_theta_front;
	end_theta_front = map_theta_front;

	if(start_theta_front > 0)
		start_theta_back = start_theta_front - pi;
	else
		start_theta_back = start_theta_front + pi;

	goal_x = x;
	goal_y = y;
	goal_theta = orientation * pi / 180;

	location_data(1);

	//forward or backward
	if(std::abs(map_theta_front - to_theta) > pi / 2 && std::abs(map_theta_front - to_theta) < 1.5 * pi)
		direction = 0;
	else
		direction = 1;

	while(!arrive){

		//spin forward
		if(((std::abs(map_theta_front - to_theta) > pi / 180) && std::abs(map_theta_front - to_theta) <= pi * 359 / 180) && (std::abs(last_x) >= 1 ||
			std::abs(last_y) >= 1) && direction && !ar){

			mode = 1;
			check_sf++;

			if(!Correction){
				if(((map_theta_front > to_theta && std::abs(map_theta_front - to_theta) < pi * 1.5)) ||
					(std::abs(map_theta_front - to_theta) > pi * 1.5 && map_theta_front < 0))
					trans_speedTransfer_modle(0, -1 * spin);
				else
					trans_speedTransfer_modle(0, spin);
			}
			else{
				spin = 0;

				if(((map_theta_front > to_theta && std::abs(map_theta_front - to_theta) < pi * 1.5)) ||
					(std::abs(map_theta_front - to_theta) > pi * 1.5 && map_theta_front < 0))
					trans_speedTransfer_modle(0, -1 * Correction_sp);
				else
					trans_speedTransfer_modle(0, Correction_sp);
			}
			arrive = 0;
		}
		//spin backward
		else if((std::abs(map_theta_back - to_theta) > pi / 180 && std::abs(map_theta_back - to_theta) <= pi * 359 / 180) && (std::abs(last_x) >= 1 ||
			std::abs(last_y) >= 1) && !direction && !ar){

			mode = 1;
			check_sb++;

			if(!Correction){
				if((map_theta_back > to_theta  && std::abs(map_theta_back - to_theta) < pi * 1.5)||
				(std::abs(map_theta_back - to_theta) > pi * 1.5 && map_theta_back < 0))
					trans_speedTransfer_modle(0, -1 * spin);
				else
					trans_speedTransfer_modle(0, spin);
			}else{

				spin = 0;

				if((map_theta_back > to_theta  && std::abs(map_theta_back - to_theta) < pi * 1.5)||
					(std::abs(map_theta_back - to_theta) > pi * 1.5 && map_theta_back < 0))
					trans_speedTransfer_modle(0, -1 * Correction_sp);
				else
					trans_speedTransfer_modle(0, Correction_sp);
			}
			arrive = 0;
		}
		//move
		else if((std::abs(last_x) > 0.1 || std::abs(last_y) > 0.1) && !ar){

			mode = 2;

			Correction = 1;

			end_theta_front = map_theta_front;

			//forward or backward
			if(std::abs(map_theta_front - to_theta) > pi / 2 && std::abs(map_theta_front - to_theta) < 1.5 * pi)
				direction = 0;
			else
				direction = 1;

			if(std::abs(end_theta_front - to_theta) <= pi / 2 || std::abs(end_theta_front - to_theta) >= pi * 3 / 2){

				check_f++;
				trans_speedTransfer_modle(SP, 0);
			}
			else{

				check_b++;
				trans_speedTransfer_modle(-1 * SP, 0);
			}
			arrive = 0;
		}
		//spin to specific orientation(front)
		else if((std::abs(map_theta_front - goal_theta) >= pi / 180 && std::abs(map_theta_front - goal_theta) <= pi * 359 / 180)
			&& orientation != 1000){

			mode = 3;

			Correction = 1;

			ar = 1;

			check_a++;

			if(std::abs(map_theta_front - goal_theta) < pi){

				if(map_theta_front > goal_theta)
					trans_speedTransfer_modle(0, -1 * spin);
				else
					trans_speedTransfer_modle(0, spin);
			}
			else{
				if(map_theta_front > goal_theta)
					trans_speedTransfer_modle(0, spin);
				else
					trans_speedTransfer_modle(0, -1 * spin);
			}

			arrive = 0;
		}
		//achieve
		else{

			mode = 4;
			check_e++;
			trans_speedTransfer_modle(0, 0);

			arrive = 1;
		}
	}
}
void move_to_line(int type, bool way){

	arrive = 0;

	float f_sp = 1.5;
	float f_spin = 1;

	if(type <= 6 || type == 8){

		//front
		if(way){

			line_find = 1;

			while(!line_check(type)){

				move_mode = 0;
				path(0);
			}
		}
		//back
		else{
			chassis_right_wheel_angspeed = -1 * f_sp;
			chassis_left_wheel_angspeed  = -1 * f_sp;

			while(!line_check(type)){}
		}
	}
	else{

		//trans
		if(way){
			chassis_right_wheel_angspeed = f_spin;
			chassis_left_wheel_angspeed  = -1 * f_spin;
		}
		//cis
		else{
			chassis_right_wheel_angspeed = -1 * f_spin;
			chassis_left_wheel_angspeed  = f_spin;
		}
		while(!line_check(type)){}
	}

	chassis_right_wheel_angspeed = 0;
	chassis_left_wheel_angspeed  = 0;
	move_mode = 2;
	line_find = 0;
	arrive = 1;
}
void ach(bool ach){
	arrive = ach;
}

void speed_change(int MODE){

	//integral
	if(MODE == 1){

		//speed up
		if(mode == 2 && (std::abs(last_x) > 20 || std::abs(last_y) > 20) && SP <= max_sp &&
				(std::abs(goal_x - start_x) > 20 || std::abs(goal_y - start_y) > 20))
			//a cm/ms
			SP += std::pow(max_sp, 2) / (2000 * 20);

		//slow down
		else if(mode == 2 && std::abs(last_x) <= 20 && std::abs(last_y) <= 20 && SP >= min_sp &&
				(std::abs(goal_x - start_x) > 20 || std::abs(goal_y - start_y) > 20))
			//a cm/ms
			SP -= (std::pow(max_sp, 2) - std::pow(min_sp, 2)) / (2000 * 20);

		else if(mode == 2 && std::abs(goal_x - start_x) <= 20 && std::abs(goal_y - start_y) <= 20)
			SP = min_sp;

		//ang speed up
		else if(mode == 1 && last_theta > std::fmin(std::abs(to_theta - start_theta_front), std::abs(to_theta - start_theta_back)) * 2  / 3 && spin <= max_spin &&
			std::fmin(std::abs(to_theta - start_theta_front), std::abs(to_theta - start_theta_back)) > pi / 18)
			//alpha rad/ms
			spin += std::pow(max_spin, 2) / (2000 * (std::fmin(std::abs(to_theta - start_theta_front), std::abs(to_theta - start_theta_back)) * 2 / 3));

			//ang speed down
		else if(mode == 1 && last_theta <= std::fmin(std::abs(to_theta - start_theta_front), std::abs(to_theta - start_theta_back)) / 3 && spin >= min_spin &&
			std::fmin(std::abs(to_theta - start_theta_front), std::abs(to_theta - start_theta_back)) > pi / 18)
			//alpha rad/ms
			spin -= (std::pow(max_spin, 2) - std::pow(min_spin, 2)) / (2000 * (std::fmin(std::abs(to_theta - start_theta_front), std::abs(to_theta - start_theta_back)) / 3));

		else if(mode == 1 && std::fmin(std::abs(to_theta - start_theta_front), std::abs(to_theta - start_theta_back)) <= pi / 18
				&& std::fmin(std::abs(to_theta - start_theta_front), std::abs(to_theta - start_theta_back)) != 0)
			spin = min_spin;

		//end ang speed up
		else if(mode == 3 && last_goal_theta > std::abs(end_theta_front - goal_theta) * 2 / 3 && spin <= max_spin &&
				std::abs(end_theta_front - goal_theta) > pi / 18)
			//alpha rad/ms
			spin += std::pow(max_spin, 2) / (2000 * std::abs(end_theta_front - goal_theta) * 2 / 3);

		//end ang speed down
		else if(mode == 3 && last_goal_theta <= std::abs(end_theta_front - goal_theta) / 3 && spin >= min_spin &&
				std::abs(end_theta_front - goal_theta) > pi / 18)
			//alpha rad/ms
			spin -= (std::pow(max_spin, 2) - std::pow(min_spin, 2)) / (2000 * std::abs(end_theta_front - goal_theta) / 3);

		else if(mode == 3 && std::abs(end_theta_front - goal_theta) <= pi / 18 &&
				std::abs(end_theta_front - goal_theta) != 0)
			spin = min_spin;

		else if(arrive){

			SP = 0;
			spin = 0;
		}
	}
	//path
	else if(MODE == 0){

		//find line
		if(line_find && !arrive){

			chassis_right_wheel_angspeed = path_motor_speed[0];
			chassis_left_wheel_angspeed  = path_motor_speed[1];
		}
		//spin
		else if(path_motor_speed[0] * path_motor_speed[1] < 0){

			chassis_right_wheel_angspeed = path_motor_speed[0];
			chassis_left_wheel_angspeed  = path_motor_speed[1];
		}
		//speed up
		else if(last_y > 15 && path_dis > 15){

			if(chassis_right_wheel_angspeed <= path_motor_speed[0])
				chassis_right_wheel_angspeed += 0.003;
			else
				chassis_right_wheel_angspeed = path_motor_speed[0];

			if(chassis_left_wheel_angspeed <= path_motor_speed[1])
				chassis_left_wheel_angspeed += 0.003;
			else
				chassis_left_wheel_angspeed = path_motor_speed[1];
		}
		//slow down
		else if(last_y < 15 && path_dis > 15){

			if(chassis_right_wheel_angspeed >= path_motor_speed[0] / 5)
				chassis_right_wheel_angspeed -= 0.003;
			else
				chassis_right_wheel_angspeed = path_motor_speed[0] / 5;

			if(chassis_left_wheel_angspeed >= path_motor_speed[1] / 5)
				chassis_left_wheel_angspeed -= 0.003;
			else
				path_motor_speed[1] = path_motor_speed[1] / 5;
		}
		else if(path_dis <= 15){

			chassis_right_wheel_angspeed = 0.4;
			chassis_left_wheel_angspeed = 0.4;
		}
		else if(arrive){

			chassis_right_wheel_angspeed = 0;
			chassis_left_wheel_angspeed  = 0;
		}
	}
	else if(MODE == 3){

	}
	//stop
	else{
		chassis_right_wheel_angspeed = 0;
		chassis_left_wheel_angspeed  = 0;
	}
}
