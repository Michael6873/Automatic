
#include <Motor.h>

#ifndef INC_TELEGA_H_
#define INC_TELEGA_H_

class Telega{

public:

	Telega():
		leftMotor(ENC_LEFT_TIM,MOT_L_PWM_TIM,CHANNEL3),
		rightMotor(ENC_RIGHT_TIM,MOT_R_PWM_TIM,CHANNEL1)
	{};

	void handler(){
			leftMotor.handler();
			rightMotor.handler();
	}
	void setRobotSpeed(float linSpeed, float angSpeed){
		setMotorSpeed(linSpeed-angSpeed,linSpeed+angSpeed);
	}

	float getCurSpL(){
		return leftMotor.getCurrentSpeed();
	}

	float getCurSpR(){
		return rightMotor.getCurrentSpeed();
	}

	float getCurEncpR(){
		return rightMotor.getEncoderValue();
	}

	float getCurEncpL(){
		return leftMotor.getEncoderValue();
	}


private:

	void setMotorSpeed(float lSpd, float rSpd){
			leftMotor.setTargetSpeed(-lSpd);
			rightMotor.setTargetSpeed(rSpd);
	}

	Motor leftMotor,rightMotor;
};



#endif /* INC_TELEGA_H_ */
