/*
 * BFilter.h
 *
 *  Created on: 19 янв. 2025 г.
 *      Author: Mihail
 */

#ifndef INC_BFILTER_H_
#define INC_BFILTER_H_


#include <vector> // Для использования std::vector (если нужно)
#include "math.h"

class BFilter {
public:
    // Конструктор по умолчанию
    BFilter() {
        // Инициализация коэффициентов фильтра нулями
        //coefs = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        initializeStates();
    }
    // Конструктор с заданными коэффициентами
    BFilter(const std::vector<float> _coefs)
    	:coefs(_coefs)
    {
//    	for(int i =0; i<5; i++){
//    		coefs[i] = _coefs[i];
//    	}
    	/*
        if (_coefs.size() == 5) {
           coefs = _coefs;
        } else {
            // Если коэффициенты заданы неверно, инициализируем нулями
            coefs = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        }
        */
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
    /*
    void setCoefficients(const std::vector<float>& _coefs) {
        if (_coefs.size() == 5) {
            coefs = _coefs;
        }
    }
    */
    // Метод для получения текущих коэффициентов фильтра
//    std::vector<float> getCoefficients() const {
//        return coefs;
//    }

	static std::vector<float> CalcFirstOrder(float freq, float sampleRate) {
		float T = 1 / sampleRate;
		float omegaC = 2 * M_PI * freq;
		float a0 = omegaC + 2 / T;
		float a1 = (omegaC - 2 / T) / a0;
		float b0 = omegaC / a0;
		std::vector<float> buf = {a1, 0, b0, b0, 0};
		return buf;
	};
	static std::vector<float> CalcSecondOrder(float freq, float sampleRate) {
		float T = 1 / sampleRate;
		float omegaC = 2 * M_PI * freq;
		float a0 = pow(omegaC, 2) + 2 * pow(2, 0.5) * omegaC / T + 4 / pow(T, 2);
		float a1 = (2 * pow(omegaC, 2) - 8 / pow(T, 2)) / a0;
		float a2 = (pow(omegaC, 2) - 2 * pow(2, 0.5) * omegaC / T + 4 / pow(T, 2)) / a0;
		float b0 = pow(omegaC, 2) / a0;
		float b1 = 2 * pow(omegaC, 2) / a0;
		float b2 = b0;
		//float buf[5] = {a1, a2, b0, b1, b2};
		std::vector<float> buf = {a1, a2, b0, b1, b2};

		return buf;
	};

private:
	std::vector<float> coefs = std::vector<float>(5, 0.0f); // Коэффициенты фильтра [a1, a2, b0, b1, b2]
    //float coefs[5] = {0.0f};
	float x1, x2;   // Предыдущие входные значения (x[n-1], x[n-2])
    float y1, y2;   // Предыдущие выходные значения (y[n-1], y[n-2])
};


#endif /* INC_BFILTER_H_ */
