
#ifndef INC_ROBOT_CMD_H_
#define INC_ROBOT_CMD_H_

#define ENC_MAX 1800 // максимальное число тиков энкодера за оборот
#define FAST_CYCLE 2 // время в мс для замера скорости двигателей

#define MAX_MOT_PWM 10000
#define MIN_MOT_PWM 1000

#define MAX_MOT_SPEED 175
#define TURN_SPEED 100

#define ZERO 0

#define MOT_PER_SIDE 2

#define ENC_SET_VALUE 20000

#define MOT_R_PWM_TIM &htim1
#define MOT_L_PWM_TIM &htim1
#define ENC_LEFT_TIM &htim1
#define ENC_RIGHT_TIM &htim1

#define CHANNEL1 1
#define CHANNEL2 2
#define CHANNEL3 3
#define CHANNEL4 4


#endif /* INC_ROBOT_CMD_H_ */
