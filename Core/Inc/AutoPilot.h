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


class AutoPilot {
public:
	AutoPilot(Navigation& _nav, LineDetector& _line, PiConnect& _rpi, PathTracker& _path)
		: nav(_nav), line(_line), rpi(_rpi), path(_path), aQueue(_path)
	{};

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
		static uint32_t enterAvoidLineMs = millis();
		if (curState != IDLE) {
			if (line.isCrossed()) {
				curState = AVOID_LINE;
			}
		}

		switch (curState)
		{
		case AutoPilot::IDLE:
			if (lastState != IDLE) {
				lastState = IDLE;
				aQueue.clear();
				aQueue.push(STOP);
				aQueue.push(ACTIONS::IDLE);
				Serial.println("Enter IDLE");
			}
			break;

		case AutoPilot::SEARCH:
			if (lastState != SEARCH) {
				lastState = SEARCH;
				aQueue.clear();
				aQueue.push(TURN_RIGHT);
				aQueue.push(DELAY, 250);
				aQueue.push(GO_FORWARD);
				aQueue.push(DELAY, 1600);
				aQueue.push(TURN_LEFT);
				aQueue.push(DELAY, 200);
				aQueue.push(TURN_RIGHT, 450);
				Serial.println("Enter SEARCH");
			}

			if (aQueue.isClear()) {
				aQueue.push(GO_FORWARD);
				aQueue.push(DELAY, 1800);
				aQueue.push(TURN_LEFT);
				aQueue.push(DELAY, 100);
				aQueue.push(TURN_RIGHT);
				aQueue.push(DELAY, 600);
				aQueue.push(TURN_LEFT);
				aQueue.push(DELAY, 100);
			}

			if (rpi.getEnemyCnt() > 0) {
				curState = ATACK;
			}
			break;

		case AutoPilot::ATACK:
			if (lastState != ATACK) {
				lastState = ATACK;
				p.robotMaxSpd = p.robotGndSpd;
				Serial.println("Enter ATACK");
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
				Serial.println("Enter FIN ATACK");
				aQueue.clear();
				aQueue.push(SET_POINT, rpi.getEnemyPose() + Vector2f(500, 0));
				aQueue.push(DELAY, 1000);
				p.robotMaxSpd = p.robotAtackSpd;
			}

			break;
		case AutoPilot::ENEMY_LOST:
			break;
		case AutoPilot::AVOID_LINE:
			if (lastState != AVOID_LINE) {
				p.robotMaxSpd = p.robotGndSpd;
				enterAvoidLineMs = millis();
				lastState = AVOID_LINE;
				Serial.println("Enter AVOID");
				aQueue.clear();
				aQueue.push(ACTIONS::GO_BACKWARD);
				aQueue.push(DELAY, 400);
				aQueue.push(ACTIONS::TURN_LEFT);
				aQueue.push(DELAY, 1000);
				aQueue.push(ACTIONS::GO_FORWARD);
				//delay(500);
			}
			if ((lastState == AVOID_LINE) && (millis() - enterAvoidLineMs > 500)) {
				//lastState = SEARCH;
			}
			if (aQueue.isClear()) {
				curState = SEARCH;
			}
			break;
		default:
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
	LineDetector& line;
	PiConnect& rpi;
	PathTracker& path;

	STATES lastState = IDLE, curState = IDLE;
};
#endif // !_AUTO_PILOT_H_













#endif /* INC_AUTOPILOT_H_ */
