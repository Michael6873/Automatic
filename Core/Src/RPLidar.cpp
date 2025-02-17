/*
 * RPLidar.cpp
 *
 *  Created on: 1 янв. 2025 г.
 *      Author: Mihail
 */
#include "RPLidar.h"
#include <limits>
#include <Robot_cmd.h>

extern UART_HandleTypeDef huart2; // Дескриптор UART1
extern uint8_t uart_rx_buffer[RX_BUFFER_SIZE];
static bool uart1_is_open = false;

// Закрытие UART1
void RPLidar::end() {
    if (isOpen()) {
        HAL_UART_DeInit(&huart2); // Деинициализация UART1
        uart1_is_open = false;    // Отмечаем UART как закрытый
    }
}

// Проверка состояния UART1 (открыт или закрыт)
bool RPLidar::isOpen() {
    return uart1_is_open; // Возвращаем текущее состояние UART1
}

RPLidar::RPLidar() {

	clearArray(distances);
	clearArray(minDist);
    _currentMeasurement.distance = 0;
    _currentMeasurement.angle = 0;
    _currentMeasurement.quality = 0;
    _currentMeasurement.startBit = 0;
}


RPLidar::~RPLidar()
{
    end();
}


// Инициализация UART1 для работы с RPLIDAR
bool RPLidar::begin() {
    // Если UART уже открыт, завершаем его работу
    if (isOpen()) {
 //       end();
    }

    // Настройка UART1 для работы с RPLIDAR
   // huart2.Instance = USART2;
   // huart2.Init.BaudRate = 115200;
   // huart2.Init.WordLength = UART_WORDLENGTH_8B;
   // huart2.Init.StopBits = UART_STOPBITS_1;
   // huart2.Init.Parity = UART_PARITY_NONE;
   // huart2.Init.Mode = UART_MODE_TX_RX;
   // huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
   // huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    // Инициализация UART1
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        return false; // Если инициализация не удалась
    }

    // Отмечаем UART как открытый
    uart1_is_open = true;
    return true;
}

uint32_t RPLidar::_sendCommand(uint8_t cmd, const void *payload, size_t payloadsize) {
    rplidar_cmd_packet_t pkt_header;
    uint8_t checksum = 0;
    HAL_StatusTypeDef status;

    // Проверяем наличие полезной нагрузки
    if (payloadsize && payload) {
        cmd |= RPLIDAR_CMDFLAG_HAS_PAYLOAD;
    }

    // Формируем заголовок команды
    pkt_header.syncByte = RPLIDAR_CMD_SYNC_BYTE;
    pkt_header.cmd_flag = cmd;

    // Отправляем заголовок (2 байта)
    status = HAL_UART_Transmit(&huart2, (uint8_t *)&pkt_header, sizeof(pkt_header), HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return RESULT_OPERATION_FAIL; // Если произошла ошибка при передаче, возвращаем ошибку
    }

    // Если есть полезная нагрузка
    if (cmd & RPLIDAR_CMDFLAG_HAS_PAYLOAD) {
        // Инициализируем контрольную сумму
        checksum ^= RPLIDAR_CMD_SYNC_BYTE;
        checksum ^= cmd;
        checksum ^= (payloadsize & 0xFF);

        // Вычисляем контрольную сумму для полезной нагрузки
        for (size_t pos = 0; pos < payloadsize; ++pos) {
            checksum ^= ((uint8_t *)payload)[pos];
        }

        // Отправляем размер полезной нагрузки (1 байт)
        uint8_t sizebyte = (uint8_t)payloadsize;
        status = HAL_UART_Transmit(&huart2, &sizebyte, 1, HAL_MAX_DELAY);
        if (status != HAL_OK) {
            return RESULT_OPERATION_FAIL; // Ошибка при передаче
        }

        // Отправляем полезную нагрузку
        status = HAL_UART_Transmit(&huart2, (uint8_t *)payload, payloadsize, HAL_MAX_DELAY);
        if (status != HAL_OK) {
            return RESULT_OPERATION_FAIL; // Ошибка при передаче
        }

        // Отправляем контрольную сумму (1 байт)
        status = HAL_UART_Transmit(&huart2, &checksum, 1, HAL_MAX_DELAY);
        if (status != HAL_OK) {
            return RESULT_OPERATION_FAIL; // Ошибка при передаче
        }
    }

    return RESULT_OK; // Если все данные успешно отправлены
}

uint32_t RPLidar::_sendCommand_IT(uint8_t cmd, const void *payload, size_t payloadsize){
    rplidar_cmd_packet_t pkt_header;
    uint8_t checksum = 0;
    HAL_StatusTypeDef status;

    // Проверяем наличие полезной нагрузки
    if (payloadsize && payload) {
        cmd |= RPLIDAR_CMDFLAG_HAS_PAYLOAD;
    }

    // Формируем заголовок команды
    pkt_header.syncByte = RPLIDAR_CMD_SYNC_BYTE;
    pkt_header.cmd_flag = cmd;

    // Отправляем заголовок (2 байта)
    status = HAL_UART_Transmit_IT(&huart2, (uint8_t *)&pkt_header, sizeof(pkt_header));
    if (status != HAL_OK) {
        return RESULT_OPERATION_FAIL; // Если произошла ошибка при передаче, возвращаем ошибку
    }

    // Если есть полезная нагрузка
    if (cmd & RPLIDAR_CMDFLAG_HAS_PAYLOAD) {
        // Инициализируем контрольную сумму
        checksum ^= RPLIDAR_CMD_SYNC_BYTE;
        checksum ^= cmd;
        checksum ^= (payloadsize & 0xFF);

        // Вычисляем контрольную сумму для полезной нагрузки
        for (size_t pos = 0; pos < payloadsize; ++pos) {
            checksum ^= ((uint8_t *)payload)[pos];
        }

        // Отправляем размер полезной нагрузки (1 байт)
        uint8_t sizebyte = (uint8_t)payloadsize;
        status = HAL_UART_Transmit_IT(&huart2, &sizebyte, 1);
        if (status != HAL_OK) {
            return RESULT_OPERATION_FAIL; // Ошибка при передаче
        }

        // Отправляем полезную нагрузку
        status = HAL_UART_Transmit_IT(&huart2, (uint8_t *)payload, payloadsize);
        if (status != HAL_OK) {
            return RESULT_OPERATION_FAIL; // Ошибка при передаче
        }

        // Отправляем контрольную сумму (1 байт)
        status = HAL_UART_Transmit_IT(&huart2, &checksum, 1);
        if (status != HAL_OK) {
            return RESULT_OPERATION_FAIL; // Ошибка при передаче
        }
    }

    return RESULT_OK; // Если все данные успешно отправлены
}


uint32_t RPLidar::_waitResponseHeader(rplidar_ans_header_t *header, uint32_t timeout) {
    uint32_t startTick = HAL_GetTick(); // Запоминаем начальное время
    uint8_t recvPos = 0; // Текущая позиция приёмного буфера
    uint8_t *headerbuf = (uint8_t *)header; // Преобразуем указатель на заголовок к массиву байтов
    uint8_t currentByte;

    while ((HAL_GetTick() - startTick) < timeout) { // Пока не истёк таймаут
        // Считываем один байт через UART
        HAL_StatusTypeDef status = HAL_UART_Receive(&huart2, &currentByte, 1, timeout - (HAL_GetTick() - startTick));
        if (status == HAL_OK) { // Если байт успешно прочитан
            switch (recvPos) {
                case 0:
                    // Проверяем первый синхронизирующий байт
                    if (currentByte != RPLIDAR_ANS_SYNC_BYTE1) {
                        continue; // Если не совпадает, игнорируем байт и продолжаем
                    }
                    break;
                case 1:
                    // Проверяем второй синхронизирующий байт
                    if (currentByte != RPLIDAR_ANS_SYNC_BYTE2) {
                        recvPos = 0; // Если не совпадает, сбрасываем позицию
                        continue;
                    }
                    break;
            }
            // Записываем байт в буфер заголовка
            headerbuf[recvPos++] = currentByte;

            // Если заголовок полностью считан
            if (recvPos == sizeof(rplidar_ans_header_t)) return RESULT_OK; // Возвращаем успешный результат
        }
        else if (status == HAL_TIMEOUT) return RESULT_OPERATION_TIMEOUT; // Если истёк таймаут
        else return RESULT_OPERATION_FAIL; // Если произошла ошибка
    }

    return RESULT_OPERATION_TIMEOUT; // Если цикл завершился без успешного завершения
}



uint32_t RPLidar::getHealth(rplidar_response_device_health_t &healthinfo, uint32_t timeout) {
    uint32_t currentTs = HAL_GetTick(); // Получаем текущее время
    uint32_t elapsedTime;

    uint8_t *infobuf = (uint8_t *)&healthinfo; // Буфер для хранения данных о состоянии
    uint8_t recvPos = 0;

    rplidar_ans_header_t response_header;
    uint32_t ans;

    // Проверяем, открыт ли UART
    if (!isOpen()) return RESULT_OPERATION_FAIL;

    // Отправляем команду на запрос состояния устройства
    if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_DEVICE_HEALTH, NULL, 0))) return ans; // Если команда не отправлена, возвращаем ошибку

    // Ожидаем заголовок ответа от устройства
    if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) return ans; // Если заголовок не получен, возвращаем ошибку

    // Проверяем, что полученный заголовок соответствует ожидаемому
    if (response_header.type != RPLIDAR_ANS_TYPE_DEVHEALTH) return RESULT_INVALID_DATA; // Неверный тип заголовка

    // Проверяем размер данных в заголовке
    if (response_header.size < sizeof(rplidar_response_device_health_t)) return RESULT_INVALID_DATA; // Неверный размер данных

    // Читаем данные о состоянии устройства
    while ((elapsedTime = HAL_GetTick() - currentTs) <= timeout) {
        uint8_t currentByte;
        HAL_StatusTypeDef status = HAL_UART_Receive(&huart2, &currentByte, 1, timeout - elapsedTime);

        if (status == HAL_OK) { // Если байт успешно прочитан
            infobuf[recvPos++] = currentByte; // Записываем байт в буфер

            // Если все данные успешно прочитаны
            if (recvPos == sizeof(rplidar_response_device_health_t)) return RESULT_OK;
        }
        else if (status == HAL_TIMEOUT) return RESULT_OPERATION_TIMEOUT; // Если истек таймаут
        else return RESULT_OPERATION_FAIL; // Если произошла ошибка
    }

    return RESULT_OPERATION_TIMEOUT; // Таймаут превышен
}

uint32_t RPLidar::getDeviceInfo(rplidar_response_device_info_t &info, uint32_t timeout) {
    uint8_t recvPos = 0;
    uint32_t startTime = HAL_GetTick(); // Начальное время
    uint8_t *infobuf = (uint8_t *)&info;
    rplidar_ans_header_t response_header;
    uint32_t ans;

    // Проверяем, открыт ли UART
    if (!isOpen()) return RESULT_OPERATION_FAIL;

    // Отправляем команду запроса информации об устройстве
    if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_DEVICE_INFO, NULL, 0))) {
        return ans;
    }

    // Ожидаем заголовок ответа от устройства
    if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
        return ans;
    }

    // Проверяем правильность типа ответа
    if (response_header.type != RPLIDAR_ANS_TYPE_DEVINFO) {
        return RESULT_INVALID_DATA;
    }

    // Проверяем, что размер ответа корректен
    if (response_header.size < sizeof(rplidar_response_device_info_t)) {
        return RESULT_INVALID_DATA;
    }

    // Чтение данных ответа
    while ((HAL_GetTick() - startTime) <= timeout) {
        uint8_t currentByte;

        // Попытка прочитать байт из UART
        if (HAL_UART_Receive(&huart2, &currentByte, 1, 10) == HAL_OK) {
            infobuf[recvPos++] = currentByte;

            // Если получен весь ответ
            if (recvPos == sizeof(rplidar_response_device_info_t)) {
                return RESULT_OK;
            }
        }
    }

    // Если время ожидания истекло
    return RESULT_OPERATION_TIMEOUT;
}

uint32_t RPLidar::stop()
{
    if (!isOpen()) return RESULT_OPERATION_FAIL;
    uint32_t ans = _sendCommand(RPLIDAR_CMD_STOP,NULL,0);
    return ans;
}

uint32_t RPLidar::startScan(bool force, uint32_t timeout) {
    uint32_t ans;

    // Проверяем, открыт ли UART
    if (!isOpen()) return RESULT_OPERATION_FAIL;

    // Останавливаем предыдущую операцию
    stop();

    // Отправляем команду на сканирование

        uint8_t command = force ? RPLIDAR_CMD_FORCE_SCAN : RPLIDAR_CMD_SCAN;
        ans = _sendCommand(command, NULL, 0);
        if (IS_FAIL(ans)) return ans;

        // Ожидание заголовка подтверждения
        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) return ans;

        // Проверяем тип заголовка
        if (response_header.type != RPLIDAR_ANS_TYPE_MEASUREMENT) return RESULT_INVALID_DATA;

        // Проверяем размер заголовка
        if (response_header.size < sizeof(rplidar_response_measurement_node_t)) return RESULT_INVALID_DATA;


    return RESULT_OK;
}

uint32_t RPLidar::startScan_IT(bool force) {
    uint32_t ans;

    // Проверяем, открыт ли UART
    if (!isOpen()) return RESULT_OPERATION_FAIL;

    // Останавливаем предыдущую операцию
    if (!isOpen()) return RESULT_OPERATION_FAIL;
    ans = _sendCommand_IT(RPLIDAR_CMD_STOP,NULL,0);
    return ans;

    // Отправляем команду на сканирование

        uint8_t command = force ? RPLIDAR_CMD_FORCE_SCAN : RPLIDAR_CMD_SCAN;
        ans = _sendCommand_IT(command, NULL, 0);
        if (IS_FAIL(ans)) return ans;

    return RESULT_OK;
}

void RPLidar::startUart_IT(){
	HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 1);
}

float* RPLidar::getDist() {  // Аргумент по умолчанию здесь не указывается
    return minDist;
}

void RPLidar::setDist(uint32_t i, float value){
	minDist[i] = value;
}

void RPLidar::clearArray( float array[361]){
    for (int i = 0; i < sizeof(array) / sizeof(array[0]); ++i) {
    	array[i] = 0.0f;
    }
}

void RPLidar::clearMinDist(){
    for (int i = 0; i < sizeof(minDist) / sizeof(minDist[0]); ++i) {
    	minDist[i] = 0.0f;
    }
}


float RPLidar::getDist(int i) {  // Аргумент по умолчанию здесь не указывается
    return minDist[i];
}
float RPLidar::constrain(int32_t value,int32_t num1,int32_t num2){
	if (value>num2) value = num2;
	if (value<num1) value = num1;
	return value;
}


void RPLidar::reWriteDist() {
    for (int angle = 0; angle < 360; angle += 9) { // Шаг изменения угла 9 градусов
        float sumDistance = 0.0f; // Сумма расстояний
        int validCount = 0;       // Счётчик валидных значений

        // Определяем диапазон для поиска минимального расстояния
        int startAngle = angle - 4;
        int endAngle = angle + 4;

        // Проверяем значения в диапазоне
        for (int currentAngle = startAngle; currentAngle <= endAngle; ++currentAngle) {
            // Обрабатываем циклический характер углов
            int wrappedAngle = currentAngle;
            if (wrappedAngle < 0) {
                wrappedAngle += 360; // Если угол отрицательный, добавляем 360
            } else if (wrappedAngle >= 360) {
                wrappedAngle -= 360; // Если угол больше или равен 360, вычитаем 360
            }

            // Проверяем, чтобы угол не выходил за пределы массива
            if (wrappedAngle >= 0 && wrappedAngle < 360) {
                float distance = distances[wrappedAngle];

                // Учитываем только валидные расстояния (> 0)
                if (distance > 0) {
                    sumDistance += distance; // Добавляем расстояние к сумме
                    ++validCount;            // Увеличиваем количество валидных значений
                }
            }
        }

        // Если хотя бы 4 расстояния валидны, записываем среднее значение
        if (validCount >= 4) {
            minDist[angle] = sumDistance / validCount; // Среднее значение
        } else {
            minDist[angle] = 0.0f; // Если недостаточно валидных значений, записываем 0
        }
    }
}

uint32_t RPLidar::waitPoint(uint32_t timeout) {
    uint32_t currentTs = HAL_GetTick(); // Получаем текущее время
    uint32_t remainingtime;
    rplidar_response_measurement_node_t node;
    uint8_t *nodebuf = (uint8_t *)&node;
    uint8_t recvPos = 0;

    while ((remainingtime = HAL_GetTick() - currentTs) <= timeout) {
        uint8_t currentbyte;
        HAL_StatusTypeDef status = HAL_UART_Receive(&huart2, &currentbyte, 1, timeout);

        // Проверяем, успешно ли получен байт
        if (status != HAL_OK) continue; // Пропускаем итерацию, если байт не был получен

        switch (recvPos) {
            case 0: // Ожидаем бит синхронизации и его инверсии
                {
                    uint8_t tmp = (currentbyte >> 1);
                    if ((tmp ^ currentbyte) & 0x1){} // Проходит проверку
                    else continue; // Если проверка не прошла, ждём следующий байт
                }
                break;

            case 1: // Ожидаем, что самый старший бит равен 1
                {
                    if (currentbyte & RPLIDAR_RESP_MEASUREMENT_CHECKBIT) {} // Проходит проверку
                    else {
                        recvPos = 0; // Сбрасываем позицию и начинаем заново
                        continue;
                    }
                }
                break;
        }

        // Сохраняем текущий байт
        nodebuf[recvPos++] = currentbyte;

        // Если все байты структуры считаны
        if (recvPos == sizeof(rplidar_response_measurement_node_t)) {
            // Вычисляем значения измерений
            _currentMeasurement.distance = node.distance_q2 / 4.0f;
            _currentMeasurement.angle = constrain((node.angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f,0,360);
            _currentMeasurement.quality = (node.sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
            _currentMeasurement.startBit = (node.sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT);

            // Сохраняем минимальное расстояние для каждого угла (от 0 до 360 градусов)
            float newAngle = _currentMeasurement.angle;
            float newDistance = _currentMeasurement.distance;
            float quality =  _currentMeasurement.quality;

				if (newAngle>=0&&newAngle<=360 &&quality>=20){
					if (newDistance>MIN_RANGE_LID&&newDistance<MAX_RANGE_LID)
						distances[(int)newAngle] = newDistance; // Сохраняем  расстояние
					else distances[(int)newAngle] = 0;
				}

            reWriteDist();
            return RESULT_OK; // Успешное завершение
        }
    }

    // Если таймаут истёк
    return RESULT_OPERATION_TIMEOUT;
}



void RPLidar::onReceive(uint8_t byte) {
    static uint8_t recvPos = 0;
    rplidar_response_measurement_node_t node;
    uint8_t *nodebuf = (uint8_t *)&node;

    switch (recvPos) {
        case 0: // Ожидаем бит синхронизации и его инверсии
        {
            uint8_t tmp = (byte >> 1);
            if ((tmp ^ byte) & 0x1) {
                // Прошли проверку, сохраняем байт
                nodebuf[recvPos++] = byte;
            } else {
                // Если проверка не прошла, ждём следующий байт
                recvPos = 0;
                return;
            }
        }
        break;

        case 1: // Ожидаем, что самый старший бит равен 1
        {
            if (byte & RPLIDAR_RESP_MEASUREMENT_CHECKBIT) {
                // Прошли проверку, сохраняем байт
                nodebuf[recvPos++] = byte;
            } else {
                // Если проверка не прошла, сбрасываем позицию
                recvPos = 0;
                return;
            }
        }
        break;

        default: // Сохраняем последующие байты
        {
            nodebuf[recvPos++] = byte;
        }
        break;
    }

    // Если все байты структуры считаны
    if (recvPos == sizeof(rplidar_response_measurement_node_t)) {
        // Вычисляем значения измерений
        _currentMeasurement.distance = node.distance_q2 / 4.0f;
        _currentMeasurement.angle = constrain((node.angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f,0,360);
        _currentMeasurement.quality = (node.sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
        _currentMeasurement.startBit = (node.sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT);

        // Сохраняем минимальное расстояние для каждого угла (от 0 до 360 градусов)
        float newAngle = _currentMeasurement.angle;
        float newDistance = _currentMeasurement.distance;
        float quality =  _currentMeasurement.quality;

			if (newAngle>=0&&newAngle<=360 &&quality>=20){
				if (newDistance>MIN_RANGE_LID&&newDistance<MAX_RANGE_LID)
					distances[(int)newAngle] = newDistance; // Сохраняем  расстояние
				else distances[(int)newAngle] = 0;
			}

        // Обновляем минимальные расстояния
        reWriteDist();

        // Сбрасываем позицию для нового пакета
        recvPos = 0;
    }
}
