
#include <RPLidar.h>
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
private:
	void clearSpd(){
		spd.lin = 0;
		spd.ang = 0;
	}

	Spd spd;
};
