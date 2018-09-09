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

#include <Arduino.h>

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "config.h"
#include "imu.h"
#include "filters.h"
#include "mpu6500.h"
#include "spi_driver.h"
#include "types.h"

#define PRINT_IMU_VALUES    0 /*!< Set this to 1 in order to print the estimated attitude */

static void IMU_CalculateAngles(mpu6500_t *mpu6500, angle_t *angle, float dt);
static float IMU_CalculateHeading(angle_t *angle, sensor_t *mag);
static void IMU_RotateV(sensor_t *v, const angle_t *angle);

bool IMU_Task(mpu6500_t *mpu6500, angle_t *angle) {
    static const float dt = MPU6500_INT_DT;

    static const float gyro_rate_lpf_fc = 2.0f;
    // Set frequency cutoff and initialize filter
    low_pass_t gyro_rate_lpf[3] = {
        { .Fc = gyro_rate_lpf_fc, .prevOutput = 0 },
        { .Fc = gyro_rate_lpf_fc, .prevOutput = 0 },
        { .Fc = gyro_rate_lpf_fc, .prevOutput = 0 },
    };

    if (MPU6500_DataReady()) {
        // Read the IMU values
        MPU6500_GetData(mpu6500);

        for (uint8_t axis = 0; axis < 3; axis++) {
            // Apply exponential smoothing to the high-pass filtered gyroscope value - this is used by the PID controller
            mpu6500->gyroRateLpf.data[axis] = FILTERS_ApplyLowPass(&gyro_rate_lpf[axis], mpu6500->gyroRate.data[axis], dt);
        }

        // Calculate the attitude
        IMU_CalculateAngles(mpu6500, angle, dt);

        return true;
    }
    return false;
}

static void IMU_CalculateAngles(mpu6500_t *mpu6500, angle_t *angle, float dt) {
    static const float gyro_cmpf_factor = 2000.0f;
    static const float invGyroComplimentaryFilterFactor = (1.0f / (gyro_cmpf_factor + 1.0f));

    static low_pass_t acc_low_pass[3] = { // Set frequency cutoff and initialize filters
        { .Fc = 50.0f, .prevOutput = 0 },
        { .Fc = 50.0f, .prevOutput = 0 },
        { .Fc = 50.0f, .prevOutput = 0 },
    };

    sensor_t accLPF; // Accelerometer values after low pass filter
    float accMagSquared = 0; // Accelerometer magnitude squared
    for (uint8_t axis = 0; axis < 3; axis++) {
        accLPF.data[axis] = FILTERS_ApplyLowPass(&acc_low_pass[axis], mpu6500->accSi.data[axis], dt); // Apply exponential smoothing: https://en.wikipedia.org/wiki/Exponential_smoothing
        accMagSquared += accLPF.data[axis] * accLPF.data[axis]; // Update magnitude
    }

    angle_t deltaAngle = { .data = { mpu6500->gyroRate.roll * dt, mpu6500->gyroRate.pitch * dt, mpu6500->gyroRate.yaw * dt } };
    IMU_RotateV(&mpu6500->accBodyFrame, &deltaAngle); // Rotate body frame according to delta angle given by gyro reading

    if (69.56f < accMagSquared && accMagSquared < 127.24f) { // Check if < 0.85G (8.34 m/s^2) or > 1.15G (11.28 m/s^2), if so we just skip new accelerometer readings
        for (uint8_t axis = 0; axis < 3; axis++)
            mpu6500->accBodyFrame.data[axis] = (mpu6500->accBodyFrame.data[axis] * gyro_cmpf_factor + accLPF.data[axis]) * invGyroComplimentaryFilterFactor; // Complimentary filter accelerometer gyro readings
    }

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -p to p (radians) - see http://en.wikipedia.org/wiki/Atan2
    // Note that the app-note assumes that the accelerometer is reading +1g when aligned with gravity, so the accelerations are inverted
#if 0 // Set to 0 to restrict roll to +-90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
    // Eq. 25 and 26
    angle->roll = atan2f(-mpu6500->accBodyFrame.Y, -mpu6500->accBodyFrame.Z);
    angle->pitch  = atan2f(mpu6500->accBodyFrame.X, sqrtf(mpu6500->accBodyFrame.Y * mpu6500->accBodyFrame.Y + mpu6500->accBodyFrame.Z * mpu6500->accBodyFrame.Z)); // Use atan2 here anyway, to prevent division by 0
#else
    // Eq. 28 and 29
    angle->roll = atan2f(-mpu6500->accBodyFrame.Y, sqrtf(mpu6500->accBodyFrame.X * mpu6500->accBodyFrame.X + mpu6500->accBodyFrame.Z * mpu6500->accBodyFrame.Z)); // Use atan2 here anyway, to prevent division by 0
    angle->pitch  = atan2f(mpu6500->accBodyFrame.X, -mpu6500->accBodyFrame.Z);
#endif

    // Use gyroscope to estimate the yaw
    static sensor_t mag = { .data = { 1.0f, 0.0f, 0.0f } }; // If no magnetometer is used, then just define a vector with a x-component only
    IMU_RotateV(&mag, &deltaAngle); // Rotate body frame according to delta angle given by the gyro reading
    angle->yaw = IMU_CalculateHeading(angle, &mag); // Get heading in radians

#if 0
    static angle_t gyroAngle;
    for (uint8_t axis = 0; axis < 3; axis++)
        gyroAngle.data[axis] += mpu6500->gyroRate.data[axis] * dt; // Gyro angle is only used for debugging

    // Make sure the data goes in the same direction
    Serial.printf("%.2f\t%.2f\t", (double)(gyroAngle.roll * RAD_TO_DEG), (double)(angle->roll * RAD_TO_DEG));
    Serial.printf("%.2f\t%.2f\t", (double)(gyroAngle.pitch * RAD_TO_DEG), (double)(angle->pitch * RAD_TO_DEG));
    Serial.printf("%.2f\t%.2f\n", (double)(gyroAngle.yaw * RAD_TO_DEG), (double)(angle->yaw * RAD_TO_DEG));
#endif

#if PRINT_IMU_VALUES
    static uint8_t counter = 0;
    if (counter++ >= 50) {
        counter = 0;
        Serial.printf("%.2f\t%.2f\t%.2f\n", (double)(angle->roll * RAD_TO_DEG), (double)(angle->pitch * RAD_TO_DEG), (double)(angle->yaw * RAD_TO_DEG));
        Serial.flush();
    }
#endif
}

// See: http://www.freescale.com/files/sensors/doc/app_note/AN4248.pdf eq. 22
static float IMU_CalculateHeading(angle_t *angle, sensor_t *mag) {
    float cosx = cosf(angle->roll);
    float sinx = sinf(angle->roll);
    float cosy = cosf(angle->pitch);
    float siny = sinf(angle->pitch);

    float Bfy = mag->Z * sinx - mag->Y * cosx;
    float Bfx = mag->X * cosy + mag->Y * siny * sinx + mag->Z * siny * cosx;
    float heading = atan2f(Bfy, Bfx); // Calculate heading in radians

    if (heading < 0) // Convert heading range to 0-2pi (0-360 degrees)
        heading += TWO_PI;
    return heading;
}

// Rotate accelerometer coordinate axis by a delta angle from gyroscope
// See: http://mathworld.wolfram.com/RotationMatrix.html,
// http://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Euler_angles_.28x-y-z_extrinsic.29_.E2.86.92_Rotation_matrix and
// https://engineering.purdue.edu/~bethel/rot2.pdf
static void IMU_RotateV(sensor_t *v, const angle_t *angle) {
    sensor_t v_tmp = *v;

    float cosx = cosf(angle->roll);
    float sinx = sinf(angle->roll);
    float cosy = cosf(angle->pitch);
    float siny = sinf(angle->pitch);
    float cosz = cosf(angle->yaw);
    float sinz = sinf(angle->yaw);

    float coszcosx = cosz * cosx;
    float sinzcosx = sinz * cosx;
    float coszsinx = sinx * cosz;
    float sinzsinx = sinx * sinz;

    float mat[3][3];
    mat[0][0] = cosz * cosy;
    mat[0][1] = -cosy * sinz;
    mat[0][2] = siny;
    mat[1][0] = sinzcosx + (coszsinx * siny);
    mat[1][1] = coszcosx - (sinzsinx * siny);
    mat[1][2] = -sinx * cosy;
    mat[2][0] = (sinzsinx) - (coszcosx * siny);
    mat[2][1] = (coszsinx) + (sinzcosx * siny);
    mat[2][2] = cosy * cosx;

    v->X = v_tmp.X * mat[0][0] + v_tmp.Y * mat[1][0] + v_tmp.Z * mat[2][0];
    v->Y = v_tmp.X * mat[0][1] + v_tmp.Y * mat[1][1] + v_tmp.Z * mat[2][1];
    v->Z = v_tmp.X * mat[0][2] + v_tmp.Y * mat[1][2] + v_tmp.Z * mat[2][2];
}
