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

#include "config.h"
#include "mpu6500.h"

#include "pins.h"
#include "spi_driver.h"

#define MPU6500_SPI_CLOCK_SLOW              (1000000U) /*!< 1 MHz */
#define MPU6500_SPI_CLOCK_FAST              (20000000U) /*!< 20 MHz */

#define MPU6500_SPI_READ_FLAG               0x80 /*!< Flag used to indicate if it is a SPI read transfer */

/** @name MPU-6500 registers */
/*@{*/
#define MPU6500_SMPLRT_DIV                  0x19 /*!< Sample Rate Divider register */
#define MPU6500_INT_PIN_CFG                 0x37 /*!< INT Pin / Bypass Enable Configuration register */
#define MPU6500_ACCEL_XOUT_H                0x3B /*!< Start of Accelerometer Measurements registers */
#define MPU6500_GYRO_XOUT_H                 0x43 /*!< Start of Gyroscope Measurements registers */
#define MPU6500_USER_CTRL                   0x6A /*!< User Control register */
#define MPU6500_PWR_MGMT_1                  0x6B /*!< Power Management 1 register */
#define MPU6500_WHO_AM_I                    0x75 /*!< Who Am I register */
/**@}*/

/**
 * @name MPU-6500 scale factors
 * See datasheet: https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-6500A-01-v1.1.pdf at page 8-9
 */
/*@{*/
#define MPU6500_GYRO_SCALE_FACTOR_250       131.0f /*!< Gyroscope scale factor of +-250 deg/s */
#define MPU6500_GYRO_SCALE_FACTOR_500       65.5f /*!< Gyroscope scale factor of +-500 deg/s */
#define MPU6500_GYRO_SCALE_FACTOR_1000      32.8f /*!< Gyroscope scale factor of +-1000 deg/s */
#define MPU6500_GYRO_SCALE_FACTOR_2000      16.4f /*!< Gyroscope scale factor of +-2000 deg/s */

#define MPU6500_ACC_SCALE_FACTOR_2          16384.0f /*!< Accelerometer scale factor of +-2 g */
#define MPU6500_ACC_SCALE_FACTOR_4          8192.0f /*!< Accelerometer scale factor of +-4 g */
#define MPU6500_ACC_SCALE_FACTOR_8          4096.0f /*!< Accelerometer scale factor of +-8 g */
#define MPU6500_ACC_SCALE_FACTOR_16         2048.0f /*!< Accelerometer scale factor of +-16 g */
/**@}*/

/**
 * @name MPU-6500 MPU-6500 identification values.
 * These are the values returned by ::MPU6500_WHO_AM_I register to indicate the type of sensor.
 */
/*@{*/
#define MPU6500_WHO_AM_I_ID                 0x70 /*!< MPU-6500 identification value */
#define MPU9250_WHO_AM_I_ID                 0x71 /*!< MPU-9250 identification value */
/**@}*/

// X-axis should be facing forward
// Y-axis should be facing to the right
// Z-axis should be facing downward
static void MPU6500_BoardOrientation(sensorRaw_t *sensorRaw) {
    sensorRaw_t sensorRawTemp = *sensorRaw;
    sensorRaw->X = sensorRawTemp.X;
    sensorRaw->Y = sensorRawTemp.Y;
    sensorRaw->Z = sensorRawTemp.Z;
}

bool MPU6500_DataReady(void) {
    return digitalRead(MPU6500_INT_PIN);
}

void MPU6500_Init(mpu6500_t *mpu6500) {
    Serial.println("Looking for MPU");

    // Configure SS pin used for the SPI communication
    pinMode(MPU6500_SS_PIN, OUTPUT);
    digitalWrite(MPU6500_SS_PIN, HIGH);

    uint8_t id = SPI_Read(MPU6500_SS_PIN, MPU6500_SPI_CLOCK_SLOW, MPU6500_WHO_AM_I | MPU6500_SPI_READ_FLAG);
    if (id == MPU6500_WHO_AM_I_ID) // Read "WHO_AM_I" register
        Serial.write("MPU-6500 found\n");
    else if (id == MPU9250_WHO_AM_I_ID)
        Serial.write("MPU-9250 found\n");
    else {
        Serial.printf("Could not find MPU-6500 or MPU-9250: 0x%02X\n", id);
        while (1) {
        }
    }

    uint8_t buf[5]; // Buffer for the SPI data
    SPI_Write(MPU6500_SS_PIN, MPU6500_SPI_CLOCK_SLOW, MPU6500_PWR_MGMT_1, 1U << 7); // Reset device, this resets all internal registers to their default values
    delay(100); // The power on reset time is specified to 100 ms. It seems to be the case with a software reset as well

    while (SPI_Read(MPU6500_SS_PIN, MPU6500_SPI_CLOCK_SLOW, MPU6500_PWR_MGMT_1 | MPU6500_SPI_READ_FLAG) & (1U << 7))
        delay(1); // Wait for the bit to clear

    SPI_Write(MPU6500_SS_PIN, MPU6500_SPI_CLOCK_SLOW, MPU6500_PWR_MGMT_1, (1U << 3) | (1U << 0)); // Disable sleep mode, disable temperature sensor and use PLL as clock reference

    /*
     * To prevent switching into I2C mode when using SPI, the I2C interface should be disabled by setting the I2C_IF_DIS configuration bit.
     * Setting this bit should be performed immediately after waiting for the time specified by the "Start-Up Time for Register Read/Write" in Section 6.3.
     */
    SPI_Write(MPU6500_SS_PIN, MPU6500_SPI_CLOCK_SLOW, MPU6500_USER_CTRL, (1U << 5) | (1U << 4)); // I2C Master mode and set I2C_IF_DIS to disable slave mode I2C bus

    buf[0] = 1000 / MPU6500_INT_FREQ_HZ - 1; // Set the sample rate in Hz - frequency = 1000/(register + 1) Hz
    buf[1] = 0x01; // Disable FSYNC and set 184 Hz Gyro filtering, 1 kHz sampling rate
    buf[2] = 1U << 3; // Set Gyro Full Scale Range to +-500 deg/s
    buf[3] = 0U << 3; // Set Accelerometer Full Scale Range to +-2 g
    buf[4] = 0x00; // 218.1 Hz Acc filtering, 1 kHz sampling rate
    SPI_Write(MPU6500_SS_PIN, MPU6500_SPI_CLOCK_SLOW, MPU6500_SMPLRT_DIV, buf, 5); // Write to all five registers at once

    // Set accelerometer and gyroscope scale factor from datasheet
    mpu6500->gyroScaleFactor = MPU6500_GYRO_SCALE_FACTOR_500;
    mpu6500->accScaleFactor = MPU6500_ACC_SCALE_FACTOR_2;

    /* Configure the INT pin */
    buf[0] = (1U << 5) | (1U << 4); // Enable LATCH_INT_EN and INT_ANYRD_2CLEAR
                                    // When this bit is equal to 1, the INT pin is held high until the interrupt is cleared
                                    // When this bit is equal to 1, interrupt status is cleared if any read operation is performed
    buf[1] = 1U << 0;               // Enable RAW_RDY_EN - When set to 1, Enable Raw Sensor Data Ready interrupt to propagate to interrupt pin
    SPI_Write(MPU6500_SS_PIN, MPU6500_SPI_CLOCK_SLOW, MPU6500_INT_PIN_CFG, buf, 2); // Write to both registers at once

    pinMode(MPU6500_INT_PIN, INPUT);

    delay(10); // Wait for sensor to stabilize

    // Initialize the body frame estimate
    memset(&mpu6500->accBodyFrame, 0, sizeof(mpu6500->accBodyFrame));
    static const uint8_t accBodyCalibLength = 100;
    for (uint8_t i = 0; i < accBodyCalibLength; i++) {
        while (!MPU6500_DataReady()); // Wait until new date is ready
        MPU6500_GetData(mpu6500);
        for (uint8_t axis = 0; axis < 3; axis++)
            mpu6500->accBodyFrame.data[axis] += mpu6500->accSi.data[axis];
    }
    for (uint8_t axis = 0; axis < 3; axis++)
        mpu6500->accBodyFrame.data[axis] /= accBodyCalibLength;

    Serial.write("Successfully configured IMU\n");
}

static bool MPU6500_CheckMinMax(int32_t *array, size_t length, int16_t maxDifference) { // Used to check that the board is not moved while calibrating
    int32_t min = array[0], max = array[0];
    for (size_t i = 1; i < length; i++) {
        if (array[i] < min)
            min = array[i];
        else if (array[i] > max)
            max = array[i];
    }
    return max - min < maxDifference;
}

static int16_t MPU6500_CalibrateSensors(sensorRaw_t *acc, sensorRaw_t *gyro, int16_t maxDifference, uint8_t calibSeconds) {
    static const uint16_t calibrationLength = calibSeconds * MPU6500_INT_FREQ_HZ; // Calibrate for 20 seconds
    int32_t sensorBuffer[6][calibrationLength] = { 0 };
    uint8_t buf[14];

    for (uint16_t i = 0; i < calibrationLength; i++) {
        while (!MPU6500_DataReady()); // Wait until new date is ready
        SPI_Read(MPU6500_SS_PIN, MPU6500_SPI_CLOCK_FAST, MPU6500_ACCEL_XOUT_H | MPU6500_SPI_READ_FLAG, buf, 14);

        // Sum up all readings
        sensorBuffer[0][i] = (int16_t)((buf[0] << 8) | buf[1]); // acc X
        sensorBuffer[1][i] = (int16_t)((buf[2] << 8) | buf[3]); // acc Y
        sensorBuffer[2][i] = (int16_t)((buf[4] << 8) | buf[5]); // acc Z

        sensorBuffer[3][i] = (int16_t)((buf[8] << 8) | buf[9]); // gyro X
        sensorBuffer[4][i] = (int16_t)((buf[10] << 8) | buf[11]); // gyro Y
        sensorBuffer[5][i] = (int16_t)((buf[12] << 8) | buf[13]); // gyro Z
    }

    for (uint8_t i = 0; i < 6; i++) {
        if (!MPU6500_CheckMinMax(sensorBuffer[i], calibrationLength, maxDifference))
            return 1; // Return error
        for (uint16_t j = 1; j < calibrationLength; j++)
            sensorBuffer[i][0] += sensorBuffer[i][j]; // Sum up all readings
    }

    for (uint8_t axis = 0; axis < 3; axis++) {
        // Get average readings
        if (acc)
            acc->data[axis] = sensorBuffer[axis][0] / calibrationLength; // Get average
        if (gyro)
            gyro->data[axis] = sensorBuffer[3 + axis][0] / calibrationLength; // Get average
    }

    // Apply board orientation
    if (acc)
        MPU6500_BoardOrientation(acc);
    if (gyro)
        MPU6500_BoardOrientation(gyro);

    return 0; // No error
}

bool MPU6500_CalibrateAcc(const mpu6500_t *mpu6500) {
    sensorRaw_t accZero;
    int16_t rcode = MPU6500_CalibrateSensors(&accZero, NULL, 1000, 1); // 1000 / 16384 ~= .061g

    if (!rcode) {
        config_t config = CONFIG_GetConfig(); // Get a copy of the configuration and indicate to the driver that we want atomic access
        config.accZero = accZero;
        config.accZero.Z += (int16_t)mpu6500->accScaleFactor; // Z-axis should be reading -1g when horizontal, so we add 1g to the value
        CONFIG_SetConfig(&config); // Write new values to EEPROM

        Serial.printf("Accelerometer zero values: %d, %d, %d\n", config.accZero.X, config.accZero.Y, config.accZero.Z);
    } else
        Serial.printf("Accelerometer calibration error: %d\n", rcode);

    return rcode;
}

bool MPU6500_CalibrateGyro(void) {
    sensorRaw_t gyroZero;
    int16_t rcode = MPU6500_CalibrateSensors(NULL, &gyroZero, 1000, 1); // 1000 / 131 ~= 7.63 deg/s

    if (!rcode) {
        config_t config = CONFIG_GetConfig(); // Get a copy of the configuration and indicate to the driver that we want atomic access
        config.gyroZero = gyroZero;
        CONFIG_SetConfig(&config); // Write new values to EEPROM

        Serial.printf("Gyroscope zero values: %d, %d, %d\n", config.gyroZero.X, config.gyroZero.Y, config.gyroZero.Z);
    } else
        Serial.printf("Gyroscope calibration error: %d\n", rcode);

    return rcode;
}

// Returns accelerometer and gyro data with zero values subtracted
void MPU6500_GetData(mpu6500_t *mpu6500) {
    uint8_t buf[14]; // Buffer for the SPI data
    SPI_Read(MPU6500_SS_PIN, MPU6500_SPI_CLOCK_FAST, MPU6500_ACCEL_XOUT_H, buf, 14);

    sensorRaw_t acc, gyro; /*!< Raw accelerometer and gyroscope readings */
    acc.X = (int16_t)((buf[0] << 8) | buf[1]);
    acc.Y = (int16_t)((buf[2] << 8) | buf[3]);
    acc.Z = (int16_t)((buf[4] << 8) | buf[5]);
    gyro.X = (int16_t)((buf[8] << 8) | buf[9]);
    gyro.Y = (int16_t)((buf[10] << 8) | buf[11]);
    gyro.Z = (int16_t)((buf[12] << 8) | buf[13]);

    MPU6500_BoardOrientation(&acc); // Apply board orientation
    MPU6500_BoardOrientation(&gyro);

#if 0
    static uint8_t counter = 0;
    if (counter++ >= 10) {
        counter = 0;
        // Acceleration should be negative when aligned with the gravity vector and gyro should follow the right hand rule
        Serial.printf("IMU: %d, %d, %d\t\t%d, %d, %d\n", acc.X, acc.Y, acc.Z, gyro.X, gyro.Y, gyro.Z);
        Serial.flush();
    }
#endif

    config_t config = CONFIG_GetConfig(); // Get a copy of the current configuration values
    for (uint8_t axis = 0; axis < 3; axis++) {
        if (abs(acc.data[axis]) > 32000)
            Serial.printf("Accelerometer[%u] saturation: %d\n", axis, acc.data[axis]);
        if (abs(gyro.data[axis]) > 32000)
            Serial.printf("Gyroscope[%u] saturation: %d\n", axis, gyro.data[axis]);
        acc.data[axis] -= config.accZero.data[axis]; // Subtract accelerometer zero values
        mpu6500->accSi.data[axis] = (float)acc.data[axis] / mpu6500->accScaleFactor * GRAVITATIONAL_ACCELERATION; // Convert to m/s^2
        gyro.data[axis] -= config.gyroZero.data[axis]; // Subtract gyro zero values
        mpu6500->gyroRate.data[axis] = (float)gyro.data[axis] / mpu6500->gyroScaleFactor * DEG_TO_RAD; // Convert to rad/s
    }
}
