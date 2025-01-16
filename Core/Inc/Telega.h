
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
	void setRobotSpeed(int32_t linSpeed, int32_t angSpeed){
		setMotorSpeed(linSpeed+angSpeed,linSpeed-angSpeed);
	}


private:

	void setMotorSpeed(int32_t lSpd, int32_t rSpd){
			leftMotor.setTargetSpeed(lSpd);
			rightMotor.setTargetSpeed(rSpd);
	}

	Motor leftMotor,rightMotor;
};



#endif /* INC_TELEGA_H_ */
