#pragma once
#ifndef _ACTIONS_QUEUE_H_
#define _ACTIONS_QUEUE_H_

#include "queue"
#include <Telega.h>
#include <RPLidar.h>

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
};

class ActionsQueue
{
public:
	
	ActionsQueue():pidl(10,0.0005,1){
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

	void fastCycle() {

		int32_t K,Y;
		if (rQueue.empty()) {
			rQueue.push(RobotInstruction(IDLE));
		}
		else if ((rQueue.size() > 1) && (rQueue.front().robotAction == IDLE)) {
			rQueue.pop();
		}
		else
		{
			RobotInstruction _curInstr = rQueue.front();
			if(IS_OK(lid.waitPoint())) ;

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
				telega.setRobotSpeed(MAX_MOT_SPEED*0.8,0);
				rQueue.pop();
				break;
			case ACTIONS::GO_BACKWARD:
				telega.setRobotSpeed(-MAX_MOT_SPEED*0.8, 0);
				rQueue.pop();
				break;
			case ACTIONS::SET_SPEED_TURN:
				K = lid.getErrorAngle();
				Y = constrain((int32_t)pidl.calculate(0, K),-MAX_MOT_PWM,MAX_MOT_PWM);
				telega.setRobotSpeed(0,Y);
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
	PID pidl;
	RPLidar lid;
	Telega telega;

	bool delayInit = false;
};
#endif // !_ACTIONS_QUEUE_H_



