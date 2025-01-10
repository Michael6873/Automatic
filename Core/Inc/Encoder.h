#pragma once
#include "stm32f4xx_hal.h"
#include <Robot_cmd.h>
#include  <cmath>

class Encoder
{
public:
	Encoder(TIM_HandleTypeDef* _encTim)
		:encTim(_encTim),time(HAL_GetTick())
	{
		//HAL_TIM_Encoder_Start(_encTim, TIM_CHANNEL_1);
	};

	void handler() {
		encoderValue = (int32_t)__HAL_TIM_GET_COUNTER(encTim);
		__HAL_TIM_SET_COUNTER(encTim, 20000);
	}

	int32_t getEncoderValue(){
		return encoderValue;
	}
	int32_t difPreviousPosition(){
		return previousPosition;
	}

	void setPreviousPosition(int32_t value){
		previousPosition = value;
	}
private:

	int32_t encoderValue; // значение энкодера
    int32_t previousPosition; // предыдущее положение
    int32_t difPosition; // разница текущего и предыдущего знаечений энкодера
	TIM_HandleTypeDef* encTim;

};

