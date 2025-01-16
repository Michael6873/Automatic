/*
 * LineDetector.h
 *
 *  Created on: Jan 13, 2025
 *      Author: Lazarev
 */

#ifndef INC_LINEDETECTOR_H_
#define INC_LINEDETECTOR_H_

#include "TCS3472.h"

class LineDetector {
public:
	LineDetector()	{};
	void handler() {
		if (HAL-GetTick() - lastMs > delayMs) {
				if ((collor.get < whiteTh) && (collor.red < redTh) && (collor.green < greenTh) && (collor.blue < blueTh)) {
					lineIsCrosed = true;
				}
				else {
					lineIsCrosed = false;
				}
				lastMs = HAL_GetTick();
			}
		}

	bool isCrossed() {
		return lineIsCrosed;
	}
private:
	uint32_t delayMs = 5;
	bool lineIsCrosed = false;
	static const uint16_t whiteTh = 0x250, redTh = 200, greenTh = 200, blueTh = 200;
	TCS3472 collor;
	uint32_t lastMs = HAL_GetTick();
};
#endif // !_LINE_DETECTOR_H
