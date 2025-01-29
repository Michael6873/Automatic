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
	LineDetector()	{
	};
	void init(){
		sens.init();
	}
	void handler() {
		if (HAL_GetTick() - lastMs > delayMs) {
			blue = sens.getColorBlue();
			white = sens.getColorWhite();
			red = sens.getColorRed();
				if (blue < blueTh) {
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
	uint32_t delayMs = 10;
	bool lineIsCrosed = false;
	static const uint16_t whiteTh = 0x250, redTh = 500, greenTh = 500, blueTh = 900;
	uint16_t white, red, green, blue;
	ColorSens sens;
	uint32_t lastMs = HAL_GetTick();
};
#endif // !_LINE_DETECTOR_H
