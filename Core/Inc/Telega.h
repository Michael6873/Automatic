
#include <Motor.h>

#ifndef INC_TELEGA_H_
#define INC_TELEGA_H_

class Telega{

public:

	Telega(){
		leftMotor(ENC_LEFT_TIM,MOT_L_PWM_TIM,CHANNEL1);
		rightMotor(ENC_RIGHT_TIM,MOT_R_PWM_TIM,CHANNEL3);
	};

	void handler(){
			leftMotor.handler();
			rightMotor.handler();
	}
	void setRobotSpeed(uint32_t linSpeed, uint32_t angSpeed){
		setMotorSpeed(linSpeed+angSpeed,linSpeed-angSpeed);
	}

private:

	void setMotorSpeed(uint32_t lSpd, uint32_t rSpd){
			leftMotor.setTargetSpeed(lSpd);
			rightMotor.setTargetSpeed(rSpd);
	}

	Motor leftMotor,rightMotor;
};



#endif /* INC_TELEGA_H_ */
