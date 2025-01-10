
#include <Motor.h>

#ifndef INC_TELEGA_H_
#define INC_TELEGA_H_

class Telega{

public:

	void handler(){
		for(int i = 0;i<MOT_PER_SIDE;i++){
			leftMotor[i].handler();
			RightMotor[i].handler();
		}
	}
	void setMotorSpeed(uint32_t){
		for(int i)
	}
private:
	Motor leftMotor[MOT_PER_SIDE] = {
		Motor(MOT_PWM_TIM,CHANNEL1),
		Motor(MOT_PWM_TIM,CHANNEL2)
	};
	Motor RightMotor[MOT_PER_SIDE] = {
		Motor(MOT_PWM_TIM,CHANNEL3),
		Motor(MOT_PWM_TIM,CHANNEL4)
	};
};



#endif /* INC_TELEGA_H_ */
