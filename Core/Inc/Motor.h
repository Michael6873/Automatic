
#include "stm32f4xx_hal.h"
#include <Robot_cmd.h>
#include <Encoder.h>
#include <BFilter.h>
#include <PID.h>
#include  <cmath>

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

	int32_t constrain(int32_t value,int32_t num1,int32_t num2){
		if (value>num2) value = num2;
		if (value<num1) value = num1;
		return value;
	}

class Motor{

public:

	Motor(TIM_HandleTypeDef* _encTim,TIM_HandleTypeDef* _ctrlTim, uint8_t _ctrlTimCh):
		ctrlTim(_ctrlTim),ctrlTimCh(_ctrlTimCh),enc(_encTim),pid(20,0.05,7)
	{};


	float getTargetSpeed(){
		return targetSpeed;
	}
	float getCurrentSpeed(){
		return currentSpeed;
	}

	int16_t getEncoderValue(){
		return enc.getEncoderValue();
	}
	void setTargetSpeed(float speed){
		targetSpeed = speed;
		targetSpeed = constrain(targetSpeed,-MAX_MOT_SPEED,MAX_MOT_SPEED);
	}
	void handler(){
		calcCurSpeedMotor();
		pid.updateCoefficients(targetSpeed);
		helpPWM = pid.calculate(targetSpeed, currentSpeed);
		setMotorPWM(constrain((int32_t)helpPWM,-MAX_MOT_PWM,MAX_MOT_PWM));
	}

private:

	void calcCurSpeedMotor(){
				enc.handler();
				currentSpeed = (((float)enc.getEncoderValue())*60.0)/(ENC_MAX*FAST_CYCLE);
				currentSpeed = filt.calc(currentSpeed);
		}
	void setMotorPWM(int32_t PWM){


		if (ctrlTimCh == CHANNEL1||ctrlTimCh == CHANNEL2){
			if (PWM>0){
				__HAL_TIM_SET_COMPARE(ctrlTim, TIM_CHANNEL_1, ZERO);
				__HAL_TIM_SET_COMPARE(ctrlTim, TIM_CHANNEL_2, PWM);
			}

			else if (PWM<0){
				PWM = abs(PWM);
				__HAL_TIM_SET_COMPARE(ctrlTim, TIM_CHANNEL_2, ZERO);
				__HAL_TIM_SET_COMPARE(ctrlTim, TIM_CHANNEL_1, PWM);
			}
		}

		if (ctrlTimCh == CHANNEL3||ctrlTimCh == CHANNEL4){
			if (PWM>0){
				__HAL_TIM_SET_COMPARE(ctrlTim, TIM_CHANNEL_3, ZERO);
				__HAL_TIM_SET_COMPARE(ctrlTim, TIM_CHANNEL_4, PWM);
			}
			else if (PWM<0){
				PWM = abs(PWM);
				__HAL_TIM_SET_COMPARE(ctrlTim, TIM_CHANNEL_4, ZERO);
				__HAL_TIM_SET_COMPARE(ctrlTim, TIM_CHANNEL_3, PWM);
			}
		}
	}

	Encoder enc;
	PID pid;
	BFilter filt = BFilter(BFilter::CalcSecondOrder(5, 1000));;

	TIM_HandleTypeDef *ctrlTim;
	uint32_t ctrlTimCh;

	float helpPWM;
	float filtSpeed;
	float currentSpeed;
	float targetSpeed;

};


#endif /* INC_MOTOR_H_ */
