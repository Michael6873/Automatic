

class PID {
public:
    PID(double kp, double ki, double kd)
        : kp(kp), ki(ki), kd(kd), prevError(0), integral(0) {
    }

    // Метод для расчета управляющего воздействия
    double calculate(double target, double current) {
        double error = target - current;
        integral += error; // Накопление интегральной ошибки
        double derivative = error - prevError; // Разница ошибок
        prevError = error;

        return kp * error + ki * integral + kd * derivative;
    }

private:
    double kp, ki, kd; // Коэффициенты PID
    double prevError; // Предыдущая ошибка
    double integral;   // интегральная ошибка
};
