
#include <Robot_cmd.h>
#include <cmath>

struct Spd{
	float lin;
	float ang;
};

class TanControl{
public:
	Spd calcTgtRobotSpds(float dist, float ang) {
	    // Минимальное ограничение на расстояние для предотвращения деления на 0
	    if (dist < 0.01f) dist = 0.01f;

	    clearSpd();

	    if (fabs(ang) > 90.0f) {
	        // Если угол больше 90 градусов, отключаем линейное движение
	        spd.lin = 0.0f;
	        spd.ang = K_ANG * ang; // Только угловое движение
	    } else {
	        // Тангенциальное управление для углов в диапазоне [-90, 90]
	        spd.lin = MAX_MOT_SPEED * tanh(dist) * cos(ang);
	        spd.ang = K_ANG * ang + spd.lin * sin(ang) / dist;
	    }
	    return spd;
	}

	int32_t getErrorAngle(float *distances){
		int32_t errorAngle = 1000;
		float minDistance = 20000.0f;
		for (int32_t i = 0; i < 360; i += 9) {

			  if(distances[i]<minDistance&&distances[i]!=0){
				  minDistance = distances[i];
				  errorAngle = i;
			  }
		  }
		  return (int32_t)errorAngle;
	}

	bool getEnemy(float *distances){
		flagEnemy = false;
		for (int32_t i = 0; i < 360; i ++) {
			if(distances[i]>MIN_RANGE_LID&&distances[i]<MAX_RANGE_LID) flagEnemy = true;
		}

		return flagEnemy;
	}
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
private:
	void clearSpd(){
		spd.lin = 0;
		spd.ang = 0;
	}
	bool flagEnemy = false;
	Spd spd;
};
