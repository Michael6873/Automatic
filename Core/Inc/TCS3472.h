
#ifndef INC_TCS3472_H_
#define INC_TCS3472_H_

#include <Robot_cmd.h>
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

class ColorSens {

public:
	ColorSens(){}

	void init(){
		uint8_t K = PON;
	  	HAL_I2C_Mem_Write(&hi2c1, TCS3472_ADDR, (ENABLE_ADDR|COMAND_BIT), 1, &K, 1,1000);
	  	HAL_Delay(100);
	    K = (PON|AEN);
	    HAL_I2C_Mem_Write(&hi2c1, TCS3472_ADDR, (ENABLE_ADDR|COMAND_BIT), 1, &K, 1,1000);
	    HAL_Delay(100);
	    K = A_TIM_VALUE154;
	    HAL_I2C_Mem_Write(&hi2c1, TCS3472_ADDR, (A_TIM_ADDR|COMAND_BIT), 1, &K, 1,1000);
	    HAL_Delay(100);
	}

	uint16_t getColorRed(){
	      HAL_I2C_Mem_Read_IT(&hi2c1, TCS3472_ADDR, RED_COLOR_ADDR|COMAND_BIT_INC , 1, RxBufferRed, 2);
	      return (((uint16_t)(RxBufferRed[1]<<8))|(uint16_t)RxBufferRed[0]);
	}
	uint16_t getColorGreen(){
	      HAL_I2C_Mem_Read_IT(&hi2c1, TCS3472_ADDR, GREEN_COLOR_ADDR|COMAND_BIT_INC , 1, RxBufferGreen, 2);
	      return (((uint16_t)(RxBufferGreen[1]<<8))|(uint16_t)RxBufferGreen[0]);
	}
	uint16_t getColorBlue(){
	      HAL_I2C_Mem_Read_IT(&hi2c1, TCS3472_ADDR, BLUE_COLOR_ADDR|COMAND_BIT_INC , 1, RxBufferBlue, 2);
	      return (((uint16_t)(RxBufferBlue[1]<<8))|(uint16_t)RxBufferBlue[0]);
	}
	uint16_t getColorWhite(){
	      HAL_I2C_Mem_Read_IT(&hi2c1, TCS3472_ADDR, WHITE_COLOR_ADDR|COMAND_BIT_INC , 1, RxBufferWhite, 2);
	      return (((uint16_t)(RxBufferWhite[1]<<8))|(uint16_t)RxBufferWhite[0]);
	}
private:
	uint8_t RxBufferRed[2];
	uint8_t RxBufferGreen[2];
	uint8_t RxBufferBlue[2];
	uint8_t RxBufferWhite[2];
};


#endif /* INC_TCS3472_H_ */
