
#include "stm32f4xx_hal.h"
#include <Robot_cmd.h>
#include <Encoder.h>
#include <PID.h>
#include  <cmath>

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_


class Motor{

public:

	Motor(TIM_HandleTypeDef* _encTim,TIM_HandleTypeDef* _ctrlTim, uint8_t _ctrlTimCh):
		ctrlTim(_ctrlTim),ctrlTimCh(_ctrlTimCh),enc(_encTim)
	{}


	int32_t getTargetSpeed(){
		return targetSpeed;
	}
	int32_t getCurrentSpeed(){
		return currentSpeed;
	}
	void setTargetSpeed(int32_t speed){
		targetSpeed = speed;
		targetSpeed = constrain(targetSpeed,-MAX_MOT_SPEED,MAX_MOT_SPEED);
	}
	void handler(){
		calcCurSpeedMotor();
		setMotorPWM(pid.calculate(targetSpeed, currentSpeed));
	}


private:

	void calcCurSpeedMotor(){
			if (HAL_GetTick()-time>=FAST_CYCLE){
				time = HAL_GetTick();
				enc.handler();
				currentSpeed = ((abs(enc.getPreviousPosition()-enc.getEncoderValue()))*60000)/(ENC_MAX*FAST_CYCLE);
				enc.setPreviousPosition(enc.getEncoderValue());
			}
		}
	void setMotorPWM(int32_t PWM){

		if (ctrlTimCh<=CHANNEL2){
			if (PWM>0){
				__HAL_TIM_SET_COMPARE(ctrlTim, CHANNEL2, ZERO);
				__HAL_TIM_SET_COMPARE(ctrlTim, CHANNEL1, PWM);
			}
			else if (PWM<0){
				PWM = abs(PWM)
				__HAL_TIM_SET_COMPARE(ctrlTim, CHANNEL1, ZERO);
				__HAL_TIM_SET_COMPARE(ctrlTim, CHANNEL2, PWM);
			}
		}

		if (ctrlTimCh>=CHANNEL3){
			if (PWM>0){
				__HAL_TIM_SET_COMPARE(ctrlTim, CHANNEL4, ZERO);
				__HAL_TIM_SET_COMPARE(ctrlTim, CHANNEL3, PWM);
			}
			else if (PWM<0){
				PWM = abs(PWM)
				__HAL_TIM_SET_COMPARE(ctrlTim, CHANNEL3, ZERO);
				__HAL_TIM_SET_COMPARE(ctrlTim, CHANNEL4, PWM);
			}
		}

	}
	int32_t constrain(int32_t value,int32_t num1,int32_t num2){
		if (value>num2) value = num2;
		if (value<num1) value = num1;
		return value;
	}

	Encoder enc;
	PID pid;

	TIM_HandleTypeDef *ctrlTim;
	uint32_t ctrlTimCh;

	int32_t time; // время для подсчета задержки
	int32_t helpSpeed;

	int32_t currentSpeed;
	int32_t targetSpeed;

};


#endif /* INC_MOTOR_H_ */
