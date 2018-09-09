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

#ifndef __TYPES_H__
#define __TYPES_H__

#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

/** Union used to store raw data from the IMU */
typedef union {
    /** Struct for accessing the values using axes names */
    struct {
        int16_t X; /*!< x-axis measurement */
        int16_t Y; /*!< y-axis measurement */
        int16_t Z; /*!< z-axis measurement */
    } __attribute__((packed));
    int16_t data[3]; /*!< Access the raw data buffer as an array  */
} sensorRaw_t;

/** Union used to store floating point data from the IMU */
typedef union {
    /** Struct for accessing the values using axes names */
    struct {
        float X; /*!< x-axis value */
        float Y; /*!< y-axis value */
        float Z; /*!< z-axis value */
    } __attribute__((packed));
    float data[3]; /*!< Access the data buffer as an array */
} sensor_t;

/**
 * Union used for the roll, pitch and yaw estimate.
 * These all follow the right hand rule.
 */
typedef union {
    /** Struct for accessing the values using their principal axes names */
    struct {
        float roll; /*!< Roll value */
        float pitch; /*!< Pitch value */
        float yaw; /*!< Yaw value */
    } __attribute__((packed));
    float data[3]; /*!< Access the data buffer as an array  */
} angle_t;

#if defined(__cplusplus)
}
#endif

#endif /* __TYPES_H__ */
