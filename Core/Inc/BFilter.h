/*
 * BFilter.h
 *
 *  Created on: 19 янв. 2025 г.
 *      Author: Mihail
 */

#ifndef INC_BFILTER_H_
#define INC_BFILTER_H_


#include <vector> // Для использования std::vector (если нужно)

class BFilter {
public:
    // Конструктор по умолчанию
    BFilter() {
        // Инициализация коэффициентов фильтра нулями
        coefs = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        initializeStates();
    }
    // Конструктор с заданными коэффициентами
    BFilter(const std::vector<float>& _coefs) {
        if (_coefs.size() == 5) {
            coefs = _coefs;
        } else {
            // Если коэффициенты заданы неверно, инициализируем нулями
            coefs = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        }
        initializeStates();
    }
    // Метод для вычисления отфильтрованного значения
    float calc(float inData) {
        // Вычисление выходного значения по разностному уравнению
        float y = coefs[2] * inData + coefs[3] * x1 + coefs[4] * x2 - coefs[0] * y1 - coefs[1] * y2;

        // Обновление состояний фильтра
        x2 = x1;
        x1 = inData;
        y2 = y1;
        y1 = y;

        return y;
    }
    // Метод для инициализации состояний фильтра
    void initializeStates() {
        x1 = 0.0f;
        x2 = 0.0f;
        y1 = 0.0f;
        y2 = 0.0f;
    }
    // Метод для установки коэффициентов фильтра
    void setCoefficients(const std::vector<float>& _coefs) {
        if (_coefs.size() == 5) {
            coefs = _coefs;
        }
    }
    // Метод для получения текущих коэффициентов фильтра
    std::vector<float> getCoefficients() const {
        return coefs;
    }
private:
    std::vector<float> coefs; // Коэффициенты фильтра [a1, a2, b0, b1, b2]
    float x1, x2;   // Предыдущие входные значения (x[n-1], x[n-2])
    float y1, y2;   // Предыдущие выходные значения (y[n-1], y[n-2])
};


#endif /* INC_BFILTER_H_ */
