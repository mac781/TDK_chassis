/*
 * communication.h
 *
 *  Created on: Sep 2, 2024
 *      Author: mac
 */

#ifndef INC_COMMUNICATION_H_
#define INC_COMMUNICATION_H_

#include "stm32f4xx_hal.h"
#include "mainpp.h"
void UART_setup();
void UART_TransmitData_Arrive_Destination_DMA(UART_HandleTypeDef *huart, bool arrive_destination);
bool UART_ReceiveData_Status_DMA(UART_HandleTypeDef *huart, float *data1, float *data2, float *data3, float *data4);
void UART_TRANSMIT();


#endif /* INC_COMMUNICATION_H_ */
