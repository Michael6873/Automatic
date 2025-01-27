/*
 * AutoPilot.h
 *
 *  Created on: Jan 13, 2025
 *      Author: Lazarev
 */

#ifndef INC_AUTOPILOT_H_
#define INC_AUTOPILOT_H_

#pragma once
#ifndef _AUTO_PILOT_H_
#define _AUTO_PILOT_H_


#include <LineDetector.h>
#include <ActionsQueue.h>

class AutoPilot {
public:
	AutoPilot()
	{
	};

	enum STATES :uint8_t
	{
		IDLE,
		SEARCH,
		ATACK,
		FIN_ATACK,
		ENEMY_LOST,
		AVOID_LINE,
	};


	void handler() {
		line.handler();
		aQueue.handler();
	}

	void reset() {
		aQueue.clear();
		curState = AutoPilot::SEARCH;
	}

	void fastCycle(){
		//		if (curState != IDLE) {
		//			if (line.isCrossed()) {
		//				curState = AVOID_LINE;
		//			}
		//		}

				ang = aQueue.checkEnemy();
				switch (curState)
				{

				case AutoPilot::SEARCH:
					if (lastState != SEARCH) {
						lastState = SEARCH;
						aQueue.clear();
					}

					if (aQueue.isClear()) {
						aQueue.push(GO_FORWARD);
						aQueue.push(DELAY,500);
						aQueue.push(STOP);
						aQueue.push(DELAY,3000);
					}

					if (aQueue.getEnemy()) {
					//	curState = ATACK;
					}
					break;

				case AutoPilot::ATACK:
					if (lastState != ATACK) {
						lastState = ATACK;
						aQueue.clear();
					}
					if(abs(ang)>30&&abs(ang)<361){
						if (aQueue.isClear()){
							aQueue.push(ACTIONS::SET_SPEED_TURN);
						}
					}
					else if (abs(ang)<30){
						curState = FIN_ATACK;
					}
					if (!aQueue.getEnemy()) {
						curState = SEARCH;
					}

					break;
				case AutoPilot::FIN_ATACK:
					if (lastState != FIN_ATACK) {
						lastState = FIN_ATACK;
						aQueue.clear();
					}
						aQueue.push(GO_FORWARD_MAX);

						if (abs(ang)>30&&abs(ang)<361||!aQueue.getEnemy()){
							curState = SEARCH;
						}

					break;
				case AutoPilot::AVOID_LINE:
					if (lastState != AVOID_LINE) {
						lastState = AVOID_LINE;
						aQueue.clear();
					}
						aQueue.push(ACTIONS::GO_BACKWARD);
						aQueue.push(DELAY, 1000);
						aQueue.push(ACTIONS::TURN_LEFT);
						aQueue.push(DELAY, 500);
						aQueue.push(ACTIONS::GO_FORWARD);
					if (aQueue.isClear()) {
						curState = SEARCH;
					}
					break;
				}

		aQueue.fastCycle();
	}

	void init(){
		aQueue.init();
		HAL_Delay(100);
		line.init();
	}


private:
	ActionsQueue aQueue;
	LineDetector line;
	int32_t ang = 0;

	STATES lastState = SEARCH, curState = SEARCH;
};
#endif // !_AUTO_PILOT_H_













#endif /* INC_AUTOPILOT_H_ */
