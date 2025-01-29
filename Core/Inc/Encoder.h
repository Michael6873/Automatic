#pragma once
#include "stm32f4xx_hal.h"
#include <Robot_cmd.h>
#include  <cmath>

class Encoder
{
public:
	Encoder(TIM_HandleTypeDef* _encTim)
		:encTim(_encTim)
	{
		//HAL_TIM_Encoder_Start(_encTim, TIM_CHANNEL_1);
	};

	void handler() {
		encoderValue = (int16_t)__HAL_TIM_GET_COUNTER(encTim);
		__HAL_TIM_SET_COUNTER(encTim, 0);
	}

	int16_t getEncoderValue(){
		return encoderValue;
	}
	int16_t difPreviousPosition(){
		return previousPosition;
	}

	void setPreviousPosition(int32_t value){
		previousPosition = value;
	}
private:

	int16_t encoderValue; // значение энкодера
    int16_t previousPosition; // предыдущее положение
    int16_t difPosition; // разница текущего и предыдущего знаечений энкодера
	TIM_HandleTypeDef* encTim;

};

