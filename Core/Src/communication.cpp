/*
 * communication.cpp
 *
 *  Created on: Sep 2, 2024
 *      Author: mac
 */
#include "communication.h"
#include "mainpp.h"
#include "stdlib.h"
#include "string.h"

extern UART_HandleTypeDef huart4;

float statusData[4] = {0, 0, 0, 0};
extern bool arrive ;

bool nextPoint = 1;
int rx,tx;

uint8_t buffer_TX[sizeof(bool)];
uint8_t buffer_RX[4 * sizeof(float)];

void UART_setup() {
    HAL_UART_Receive_DMA(&huart4, buffer_RX, sizeof(buffer_RX));
}

void UART_TransmitData_Arrive_Destination_DMA(UART_HandleTypeDef *huart, bool arrive_destination) {
    memcpy(buffer_TX, &arrive_destination, sizeof(bool));
    HAL_UART_Transmit_DMA(huart, buffer_TX, sizeof(buffer_TX));
}
//void UART_ReceiveData_Status_IT(UART_HandleTypeDef *huart, float *data1, float *data2, float *data3, float *data4) {
//    memcpy(data1, buffer_RX, sizeof(float));
//    memcpy(data2, buffer_RX + sizeof(float), sizeof(float));
//    memcpy(data3, buffer_RX + 2 * sizeof(float), sizeof(float));
//    memcpy(data4, buffer_RX + 3 * sizeof(float), sizeof(float));
//    HAL_UART_Receive_IT(huart, buffer_RX, sizeof(buffer_RX));
//}
bool UART_ReceiveData_Status_DMA(UART_HandleTypeDef *huart, float *data1, float *data2, float *data3, float *data4) {
    if (huart == NULL || data1 == NULL || data2 == NULL || data3 == NULL || data4 == NULL) {
        return false; // Return false if any pointer is invalid
    }

    // Attempt to receive data
    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(huart, (uint8_t *)buffer_RX,  sizeof(buffer_RX));
    if (status != HAL_OK) {
        return false; // Return false if the reception fails
    }

    memcpy(data1, buffer_RX, sizeof(float));
    memcpy(data2, buffer_RX + sizeof(float), sizeof(float));
    memcpy(data3, buffer_RX + 2 * sizeof(float), sizeof(float));
    memcpy(data4, buffer_RX + 3 * sizeof(float), sizeof(float));
    HAL_UART_Receive_DMA(huart, buffer_RX, sizeof(buffer_RX));
    return true;
}


void UART_TRANSMIT(){
	UART_TransmitData_Arrive_Destination_DMA(&huart4, arrive);
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART4) {
    	tx++;
        // Add transmission complete logic here
    	if (arrive == true){
    			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    	}else {
    			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    	}
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

    if (huart->Instance == UART4) {

    	float t_0 = statusData[0], t_1 = statusData[1], t_2 = statusData[2], t_3 = statusData[3];

        if(UART_ReceiveData_Status_DMA(&huart4, &statusData[0], &statusData[1], &statusData[2], &statusData[3])) rx++;
        else rx--;

//        if (statusData[0] == 0 && statusData[1] == 0 && statusData[2] == 0 && statusData[3] == 0){
//        		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//        }else if (statusData[0] > 0 && statusData[1] > 0 && statusData[2] > 0 && statusData[3] > 0){
//        		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
//        }

        if(!(t_0 == statusData[0] && t_1 == statusData[1] && t_2 == statusData[2] && t_3 == statusData[3])){

        	nextPoint = 1;
        	arrive = 0;
        	UART_TRANSMIT();
        }
    }
}
