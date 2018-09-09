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

#include <EEPROM.h>

#include "config.h"

static config_t cfg;
static const typeof(cfg.configVersion) configVersion = 1; // Must be bumped every time config_t is changed

bool CONFIG_Init(void) {
    EEPROM.get(0, cfg); // Read config from EEPROM
    if (cfg.configVersion != configVersion) {
        CONFIG_SetDefault();
        //beepLongBuzzer();
        Serial.write("EEPROM values reset\n");
    }

    Serial.printf("Accelerometer zero values: %d\t%d\t%d\n", cfg.accZero.X, cfg.accZero.Y, cfg.accZero.Z);
    Serial.printf("Gyro zero values: %d\t%d\t%d\n", cfg.gyroZero.X, cfg.gyroZero.Y, cfg.gyroZero.Z);

    bool accCalibrated = true;
    if (cfg.accZero.X == 0 && cfg.accZero.Y == 0 && cfg.accZero.Z == 0) {
        Serial.write("Accelerometer is NOT calibrated\n");
        accCalibrated = false;
    }
    if (cfg.gyroZero.X == 0 && cfg.gyroZero.Y == 0 && cfg.gyroZero.Z == 0) {
        Serial.write("Gyroscope is NOT calibrated!\n");
    }

    Serial.printf("targetAngle: %.2f\n", cfg.targetAngle);

    PID_PrintValues("roll", &cfg.pid_values);

    return accCalibrated;
}

void CONFIG_SetDefault(void) {
    cfg.configVersion = configVersion; // Set the current version

    for (uint8_t axis = 0; axis < 3; axis++) {
        cfg.accZero.data[axis] = 0;
        cfg.gyroZero.data[axis] = 0;
    }
#if 0 // Current mode
    cfg.pid_values.Kp = 1.40f;
    cfg.pid_values.Ki = 0.80f;
    cfg.pid_values.Kd = 0.05f;
    cfg.pid_values.integrationLimit = 50.0f;
    cfg.pid_values.Fc = 0.0f; // Not used, as the roll rate is given as input
#else // Duty cycle mode
    cfg.pid_values.Kp = 2.10f;
    cfg.pid_values.Ki = 0.10f;
    cfg.pid_values.Kd = 0.10f;
    cfg.pid_values.integrationLimit = 5.0f;
    cfg.pid_values.Fc = 0.0f; // Not used, as the roll rate is given as input
#endif
    cfg.targetAngle = 0;
    cfg.configureBtModule = true;

    CONFIG_SetConfig(&cfg);
}

config_t CONFIG_GetConfig(void) {
    return cfg; // Return a copy of the configuration
}

void CONFIG_SetConfig(const config_t *config) {
    cfg = *config; // Update the new values

    // Write the new configuration to the EEPROM
    EEPROM.put(0, cfg);
}
