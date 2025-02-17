/*
 * RPLidar.h
 *
 *  Created on: 1 янв. 2025 г.
 *      Author: Mihail
 */

#ifndef INC_RPLIDAR_H_
#define INC_RPLIDAR_H_



/*
 * RoboPeak RPLIDAR Driver for Arduino
 * RoboPeak.com
 *
 * Copyright (c) 2014, RoboPeak
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once

#include "stm32f4xx_hal.h"
#include "RPlidar_cmd.h"
#include <float.h>
#include <string.h>

struct RPLidarMeasurement
{
    float distance;
    float angle;
    uint8_t quality;
    bool  startBit;
};

class RPLidar
{
public:
    enum {
        RPLIDAR_SERIAL_BAUDRATE = 115200,
        RPLIDAR_DEFAULT_TIMEOUT = 500,
    };

    RPLidar();
    ~RPLidar();

    // open the given serial interface and try to connect to the RPLIDAR

    bool begin();

    // close the currently opened serial interface
    void end();

    // check whether the serial interface is opened
    bool isOpen();

    // ask the RPLIDAR for its health info
    uint32_t getHealth(rplidar_response_device_health_t & healthinfo, uint32_t timeout = RPLIDAR_DEFAULT_TIMEOUT);

    // ask the RPLIDAR for its device info like the serial number
    uint32_t getDeviceInfo(rplidar_response_device_info_t & info, uint32_t timeout = RPLIDAR_DEFAULT_TIMEOUT);

    // stop the measurement operation
    uint32_t stop();

    // start the measurement operation
    uint32_t startScan(bool force = false, uint32_t timeout = RPLIDAR_DEFAULT_TIMEOUT*2);

    uint32_t startScan_IT(bool force);

    // wait for one sample point to arrive
    uint32_t waitPoint(uint32_t timeout = RPLIDAR_DEFAULT_TIMEOUT);

    void onReceive(uint8_t byte);

    float getDist(int i);
    // retrieve currently received sample point

    float* getDist();

    void clearArray( float array[361]);

    void clearMinDist();

    void startUart_IT();

    void setDist(uint32_t i, float value);

    const RPLidarMeasurement & getCurrentPoint()
    {
        return _currentMeasurement;
    }

protected:
    void reWriteDist();
    uint32_t _sendCommand(uint8_t cmd, const void * payload, size_t payloadsize);
    uint32_t _sendCommand_IT(uint8_t cmd, const void *payload, size_t payloadsize);
    uint32_t _waitResponseHeader(rplidar_ans_header_t * header, uint32_t timeout);
    float constrain(int32_t value,int32_t num1,int32_t num2);

protected:
    RPLidarMeasurement _currentMeasurement;
    float distances[361]; // Массив расстояний для каждого угла
    float minDist[361]; // Массив расстояний для каждого угла
};


#endif /* INC_RPLIDAR_H_ */
