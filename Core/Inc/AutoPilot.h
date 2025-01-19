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
	AutoPilot(Navigation& _nav, PiConnect& _rpi, PathTracker& _path)
		: nav(_nav), rpi(_rpi), path(_path), aQueue(_path)
	{
		line.init();
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
		if (curState != IDLE) {
			if (line.isCrossed()) {
				curState = AVOID_LINE;
			}
		}

		switch (curState)
		{

		case AutoPilot::SEARCH:
			if (lastState != SEARCH) {
				lastState = SEARCH;
			}
				aQueue.push(GO_FORWARD);


			if (rpi.getEnemyCnt() > 0) {
				curState = ATACK;
			}
			break;

		case AutoPilot::ATACK:
			if (lastState != ATACK) {
				lastState = ATACK;
			}

			if (rpi.getEnemyPose().norm() < 200) {
				curState = FIN_ATACK;
			}
			else if (rpi.getEnemyCnt() > 0) {
				aQueue.clear();
				aQueue.push(SET_POINT, rpi.getEnemyPose());
			}
			else {
				curState = SEARCH;
			}

			break;
		case AutoPilot::FIN_ATACK:
			if (lastState != FIN_ATACK) {
				lastState = FIN_ATACK;
				aQueue.clear();
				aQueue.push(GO_FORWARD_MAX);
				aQueue.push(DELAY, 3000);
			}

			break;
		case AutoPilot::ENEMY_LOST:
			if (lastState != ENEMY_LOST) {
				lastState = ENEMY_LOST;
				aQueue.clear();
			}

			if (aQueue.isClear()) {
				curState = SEARCH;
			}
			break;
		case AutoPilot::AVOID_LINE:
			if (lastState != AVOID_LINE) {
				lastState = AVOID_LINE;
				aQueue.clear();
				aQueue.push(ACTIONS::GO_BACKWARD);
				aQueue.push(DELAY, 400);
				aQueue.push(ACTIONS::TURN_LEFT);
				aQueue.push(DELAY, 500);
				aQueue.push(ACTIONS::GO_FORWARD);
			}
			if (aQueue.isClear()) {
				curState = SEARCH;
			}
			break;
		}

		aQueue.handler();
	}

	void reset() {
		aQueue.clear();
		curState = AutoPilot::SEARCH;
	}


private:
	ActionsQueue aQueue;
	Navigation& nav;
	LineDetector line;
	PiConnect& rpi;
	PathTracker& path;

	STATES lastState = SEARCH, curState = SEARCH;
};
#endif // !_AUTO_PILOT_H_













#endif /* INC_AUTOPILOT_H_ */
