
#include <Robot_cmd.h>
#include <cmath>

struct Spd{
	int32_t lin;
	int32_t ang;
};

class TanControl{
public:
	Spd calcTgtRobotSpds(int32_t dist, int32_t ang) {
		clearSpd();
		spd.lin = MAX_MOT_SPEED * tanh(dist) * cos(ang);
		spd.ang = K_ANG * ang + spd.lin * sin(ang) / dist;
		return spd;
	}

	int32_t getErrorAngle(float *distances){
		int32_t errorAngle = 0;
		int32_t minDistance = 10000;
		  for(int i = 0;i<=360;i++){

			  if(distances[i]<minDistance){
				  minDistance = distances[i];
				  errorAngle = limitAng(i);
			  }
		  }
		  return errorAngle;
	}
private:
	int32_t limitAng(int32_t ang){
		int32_t inAng = ang;
		while (inAng >= HALF_CIRCLE) {
			inAng -= 2 * HALF_CIRCLE;
		}
		while (inAng <= -HALF_CIRCLE) {
			inAng += 2 * HALF_CIRCLE;
		}
		return inAng;
	}
	void clearSpd(){
		spd.lin = 0;
		spd.ang = 0;
	}

	Spd spd;
};
