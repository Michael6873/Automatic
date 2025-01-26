#pragma once
#ifndef _ACTIONS_QUEUE_H_
#define _ACTIONS_QUEUE_H_

#include "queue"
#include <Telega.h>
#include <RPLidar.h>
#include <TanControl.h>

enum ACTIONS
{
	IDLE,
	STOP,
	TURN_LEFT,
	TURN_RIGHT,
	GO_FORWARD,
	GO_BACKWARD,
	SET_SPEED_TURN,
	DELAY,
	GO_FORWARD_MAX,
};

class ActionsQueue
{
public:
	
	ActionsQueue(){
	};
	void init(){
		lid.begin();
		lid.startScan();
	}

	void push(ACTIONS action) {
		rQueue.push(RobotInstruction(action));
	};
	void push(ACTIONS action, float value) {
		rQueue.push(RobotInstruction(action, value));
	};

	int getSize() {
		return rQueue.size();
	}

	void clear() {
		while (!rQueue.empty()) {
			rQueue.pop();
		}
		delayInit = false;
	}
	bool isClear() {
		if (rQueue.size() == 0) {
			return true;
		}
		else if (rQueue.front().robotAction == ACTIONS::IDLE) {
			return true;
		}
		else {
			return false;
		};
	}

	void handler(){
		telega.handler();
	}

	void onReceive(uint8_t byte){
		lid.onReceive(byte);
	}

	int32_t checkEnemy(){
		ang  = tan.getErrorAngle(lid.getDist());
		int32_t angLimit = 0;
		if (ang) angLimit = tan.limitAng(ang);
		else angLimit = 0;
		return angLimit;
	}

	bool getEnemy(){
		return tan.getEnemy(lid.getDist());
	}
	void fastCycle() {

		if(IS_OK(lid.waitPoint())) ;
		int32_t angLimit = 0;
		if (rQueue.empty()) {
			rQueue.push(RobotInstruction(IDLE));
		}
		else if ((rQueue.size() > 1) && (rQueue.front().robotAction == IDLE)) {
			rQueue.pop();
		}
		else
		{
			RobotInstruction _curInstr = rQueue.front();

			switch (_curInstr.robotAction)
			{
			case ACTIONS::IDLE:
				break;
			case ACTIONS::STOP:
				telega.setRobotSpeed(0,0);
				rQueue.pop();
				break;
			case ACTIONS::TURN_LEFT:
				telega.setRobotSpeed(0,TURN_SPEED);
				rQueue.pop();
				break;
			case ACTIONS::TURN_RIGHT:
				telega.setRobotSpeed(0,-TURN_SPEED);
				rQueue.pop();
				break;
			case ACTIONS::GO_FORWARD:
				telega.setRobotSpeed(MAX_MOT_SPEED*0.7,0);
				rQueue.pop();
				break;
			case ACTIONS::GO_FORWARD_MAX:
				telega.setRobotSpeed(MAX_MOT_SPEED,0);
				rQueue.pop();
				break;
			case ACTIONS::GO_BACKWARD:
				telega.setRobotSpeed(-MAX_MOT_SPEED*0.7, 0);
				rQueue.pop();
				break;
			case ACTIONS::SET_SPEED_TURN:
			        angLimit = checkEnemy(); // Ограничение угла [-180, 180]
				    spd.lin = 0.0f;
				    spd.ang = TURN_SPEED*angLimit/fabs((float)angLimit); // Только угловое движение
			        telega.setRobotSpeed(spd.lin, -spd.ang); // Угловая скорость инвертирована
				break;
			case ACTIONS::DELAY:
				static uint32_t delayBegin = 0;
				if (!delayInit) {
					delayInit = true;
					delayBegin = HAL_GetTick();
				}
				else if (HAL_GetTick() - delayBegin >= (uint16_t)_curInstr.parametr) {
					rQueue.pop();
					delayInit = false;
				}
				break;
			default:
				break;
			}
		}
	};

private:	
	struct RobotInstruction
	{
		ACTIONS robotAction = ACTIONS::IDLE;
		bool paramAvalible = false;
		bool pointAvalible = false;
		float parametr = 0;

		RobotInstruction() {};
		RobotInstruction(ACTIONS _action)
			: robotAction(_action) {};

		RobotInstruction(ACTIONS _action, float _param)
			: robotAction(_action), paramAvalible(true), parametr(_param), pointAvalible(false) {};

		RobotInstruction& operator=(const RobotInstruction& _instr) {
			robotAction = _instr.robotAction;
			paramAvalible = _instr.paramAvalible;
			pointAvalible = _instr.pointAvalible;
			parametr = _instr.parametr;
		}
	};
	std::queue<RobotInstruction> rQueue;
	RPLidar lid;
	Telega telega;
	TanControl tan;
	Spd spd;

	bool delayInit = false;
	int32_t ang = 0;
};
#endif // !_ACTIONS_QUEUE_H_



