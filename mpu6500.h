/* Copyright (C) 2018 Kristian Sloth Lauszus. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Sloth Lauszus
 Web      :  http://www.lauszus.com
 e-mail   :  lauszus@gmail.com
*/

#ifndef __MPU6500_H__
#define __MPU6500_H__

#include <stdbool.h>

#include "types.h"

#define MPU6500_INT_FREQ_HZ                 (1000U) /*!< Interrupt at 500 Hz */
#define MPU6500_INT_DT                      (1.0f / (float)MPU6500_INT_FREQ_HZ)

#define GRAVITATIONAL_ACCELERATION          (9.80665f) // https://en.wikipedia.org/wiki/Gravitational_acceleration

/** Struct for MPU-6500 data */
typedef struct {
    sensor_t accBodyFrame; /*!< Magnitude of the acceleration in the body frame in m/s^2 */
    float accScaleFactor; /*!< Accelerometer scale factor */
    float gyroScaleFactor; /*!< Gyroscope scale factor */
    angle_t gyroRate; /*!< Gyroscope readings in rad/s */
    angle_t gyroRateLpf; /*!< Low pass filtered gyroscope readings in rad/s */
    sensor_t accSi; /*!< Accelerometer readings in m/s^2 */
} mpu6500_t;

void MPU6500_Init(mpu6500_t *mpu6500);

bool MPU6500_DataReady(void);

void MPU6500_GetData(mpu6500_t *mpu6500);

bool MPU6500_CalibrateAcc(const mpu6500_t *mpu6500);

bool MPU6500_CalibrateGyro(void);

#endif /* __MPU6500_H__ */
