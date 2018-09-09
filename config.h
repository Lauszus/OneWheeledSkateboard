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

#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <stdbool.h>

#include "pid.h"
#include "types.h"

/** Struct with all the configurations values stored in the EEPROM */
typedef struct {
    uint32_t configVersion;
    pid_values_t pid_values;
    sensorRaw_t accZero; // Accelerometer calibration values
    sensorRaw_t gyroZero; // Gyroscope calibration values
    float targetAngle; // Resting angle of the robot
    bool configureBtModule; // Used in order to configure the Bluetooth module the first time it is powered on
} config_t;

/** Initialize the EEPROM and read the configurations structure from the EEPROM */
bool CONFIG_Init(void);

/** Set default configurations values and store them in the EEPROM */
void CONFIG_SetDefault(void);

config_t CONFIG_GetConfig(void);

/**
 * Write the configuration values to the EEPROM.
 * @param config Pointer to the configuration to write.
 */
void CONFIG_SetConfig(const config_t *config);

#endif /* __CONFIG_H__ */
