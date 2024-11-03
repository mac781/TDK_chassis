/*
 * location.h
 *
 *  Created on: Sep 2, 2024
 *      Author: mac
 */

#ifndef INC_LOCATION_H_
#define INC_LOCATION_H_

#include "motor.h"

//goto (x,y), orientation 180 ~ -180 degree，input 1000 to not spin
void integral_moveto(float x, float y, float orientation);

//update location info
void location_data(int MODE);

//0 path, 1 integral, 2 stop
void speed_change(int MODE);

void location_reset(float x, float y, float ang);

void ach(bool ach);

//1:front find line, 2:middle find line, 3:find cross road, 4:find line
void move_to_line(int type, bool way);

#endif /* INC_LOCATION_H_ */
