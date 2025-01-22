
#ifndef INC_ROBOT_CMD_H_
#define INC_ROBOT_CMD_H_

#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

#define ENC_MAX 1800 // максимальное число тиков энкодера за оборот
#define FAST_CYCLE 1.0/1000 // время в мс для замера скорости двигателей

#define MAX_MOT_PWM 10000
#define MIN_MOT_PWM 500

#define MAX_MOT_SPEED 100
#define TURN_SPEED 100

#define K_ANG 0.5
#define HALF_CIRCLE 180

#define ZERO 0

#define MOT_PER_SIDE 2

#define ENC_SET_VALUE 20000

#define MOT_R_PWM_TIM &htim3
#define MOT_L_PWM_TIM &htim3
#define ENC_LEFT_TIM &htim2
#define ENC_RIGHT_TIM &htim4

#define CHANNEL1 1
#define CHANNEL2 2
#define CHANNEL3 3
#define CHANNEL4 4

#define TCS3472_ADDR 0x29<<1
#define ENABLE_ADDR 0x00
#define COMAND_BIT 0x80
#define WHITE_COLOR_ADDR 0x14
#define RED_COLOR_ADDR 0x16
#define GREEN_COLOR_ADDR 0x18
#define BLUE_COLOR_ADDR 0x1A
#define COMAND_BIT_INC 0xA0
#define PON 0x01
#define AEN 0x02
#define A_TIM_ADDR 0x01
#define A_TIM_VALUE154 0x00

#endif /* INC_ROBOT_CMD_H_ */
