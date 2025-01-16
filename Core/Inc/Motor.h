
#include "stm32f4xx_hal.h"
#include <Robot_cmd.h>
#include <Encoder.h>
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
		ctrlTim(_ctrlTim),ctrlTimCh(_ctrlTimCh),enc(_encTim),pid(10,0.0005,1)
	{};


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
		pidClear();
		setMotorPWM(pid.calculate(targetSpeed, currentSpeed));
	}

	void pidClear(){
		static bool helpFlag = true;
		bool targFlag = (bool)targetSpeed;
		if(helpFlag!=targFlag) {
			pid.clear();
			helpFlag = targFlag;
		}
	}

private:

	void calcCurSpeedMotor(){
				enc.handler();
				currentSpeed = constrain(((ENC_SET_VALUE-enc.getEncoderValue())*60000)/(ENC_MAX*FAST_CYCLE),-MAX_MOT_SPEED,MAX_MOT_SPEED);
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

	TIM_HandleTypeDef *ctrlTim;
	uint32_t ctrlTimCh;

	int32_t time; // время для подсчета задержки
	int32_t helpSpeed;

	int32_t currentSpeed;
	int32_t targetSpeed;

};


#endif /* INC_MOTOR_H_ */
