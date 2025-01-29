

class PID {
public:
    PID(float kp, float ki, float kd)
        : kp(kp), ki(ki), kd(kd), prevError(0), integral(0) {
    }

    // Метод для расчета управляющего воздействия
    float calculate(float target, float current) {
    	float error = target - current;
        integral += error; // Накопление интегральной ошибки
        float derivative = error - prevError; // Разница ошибок
        prevError = error;

        return kp * error + ki * integral + kd * derivative;
    }
    void clear(){
    	prevError = 0;
    	integral = 0;
    }

    // Метод для обновления коэффициентов по формулам
    void updateCoefficients(double targetSpeed) {
        // Коэффициенты для низких скоростей (до 50 об/мин)
        double kp_low = 20;
        double ki_low = 0.05;
        double kd_low = 7;

        // Коэффициенты для высоких скоростей (свыше 150 об/мин)
        double kp_high = 16;
        double ki_high = 0.1;
        double kd_high = 6;

        // Линейная интерполяция коэффициентов
        if (targetSpeed <= 50) {
            // Если скорость <= 50, используем коэффициенты для низких скоростей
            kp = kp_low;
            ki = ki_low;
            kd = kd_low;
        } else if (targetSpeed >= 150) {
            // Если скорость >= 150, используем коэффициенты для высоких скоростей
            kp = kp_high;
            ki = ki_high;
            kd = kd_high;
        } else {
            // Линейная интерполяция для скоростей от 50 до 150 об/мин
            double t = (targetSpeed - 50) / (150 - 50); // Нормализация скорости в диапазоне [0, 1]
            kp = kp_low + t * (kp_high - kp_low);
            ki = ki_low + t * (ki_high - ki_low);
            kd = kd_low + t * (kd_high - kd_low);
        }
    }

private:
    float kp, ki, kd; // Коэффициенты PID
    float prevError; // Предыдущая ошибка
    float integral;   // интегральная ошибка


};
