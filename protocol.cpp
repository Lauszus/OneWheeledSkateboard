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
#include "pid.h"
#include "protocol.h"
#include "vesc.h"

#define DEBUG_PROTOCOL 1

#if 0
enum {
    SET_PID = 0,
    GET_PID,
    SET_TARGET,
    GET_TARGET,
    SEND_INFO,
    CAL_ACC,
    RESTORE_DEFAULTS,
};
#else
enum {
    SET_PID_ROLL_PITCH = 0,
    GET_PID_ROLL_PITCH,
    SET_PID_YAW,
    GET_PID_YAW,
    SET_PID_SONAR_ALT_HOLD,
    GET_PID_SONAR_ALT_HOLD,
    SET_PID_BARO_ALT_HOLD,
    GET_PID_BARO_ALT_HOLD,
    SET_SETTINGS,
    GET_SETTINGS,
    SEND_ANGLES,
    CAL_ACC,
    CAL_MAG,
    RESTORE_DEFAULTS,

    SET_PID = SET_PID_ROLL_PITCH,
    GET_PID = GET_PID_ROLL_PITCH,
};
#endif

typedef struct {
    uint8_t cmd;
    uint8_t length;
} __attribute__((packed)) msg_t;

typedef struct {
    uint16_t Kp, Ki, Kd; // Kp is multiplied by 1000, Ki multiplied by 100 and Kd are multiplied by 100000
    uint16_t integrationLimit; // Integration limit multiplied by 100
} __attribute__((packed)) pid_values_bt_t;

/*struct target_t {
    int16_t targetAngle;
} __attribute__((packed)) target;*/

struct info_t {
    int16_t duty;
    int16_t current;
    uint16_t battery;
} __attribute__((packed)) info;

//static uint8_t sendInfo; // Non-zero if values should be sent

static const char *commandHeader = "$S>";
static const char *responseHeader = "$S<";

static bool PROTOCOL_GetData(msg_t msg, uint8_t *data);
static void PROTOCOL_SendData(msg_t msg, uint8_t *data);

static void PROTOCOL_PrintResponse(void) {
    if (Serial1.available()) {
        while (Serial1.available())
            Serial.write(Serial1.read());
        Serial.println();
    }
}

void PROTOCOL_Init(void) {
    // Automatically configure the Bluetooth module the first time it is turned on
    config_t config = CONFIG_GetConfig();
    if (config.configureBtModule) {
        config.configureBtModule = false;
        CONFIG_SetConfig(&config);

        uint32_t baud_rates[] = { 9600, 57600, 115200 };
        for (uint8_t i = 0; i < sizeof(baud_rates)/sizeof(baud_rates[0]); i++) {
            Serial1.begin(baud_rates[i]);
            Serial1.println("AT+NAMESkateboard"); // Set the name to 'Skateboard'
            delay(1000); // The module waits 1 s before it considers a message complete
            PROTOCOL_PrintResponse();
            Serial1.println("AT+PIN0000"); // Set the pin to '0000'
            delay(1000); // The module waits 1 s before it considers a message complete
            PROTOCOL_PrintResponse();
            //Serial1.println("AT+BAUD7"); // Finally change the baud rate to '57600'
            Serial1.println("AT+BAUD8"); // Finally change the baud rate to '115200'
            delay(1000); // The module waits 1 s before it considers a message complete
            PROTOCOL_PrintResponse();
        }

        Serial.write("Bluetooth module configured\n");
    }
    Serial1.begin(57600);
    Serial1.setTimeout(10); // Only wait 10ms for serial data
}

void PROTOCOL_ParseData(const mpu6500_t *mpu6500, const angle_t *angle) {
    msg_t msg;
    if (Serial1.available() >= (int) strlen(commandHeader)) {
        if (Serial1.find((char*) commandHeader)) {
            if (Serial1.readBytes((uint8_t*) &msg, sizeof(msg))
                    == sizeof(msg)) {
                switch (msg.cmd) {
                case SET_PID:
                    if (msg.length == sizeof(pid_values_bt_t)) { // Make sure that it has the right length
                        pid_values_bt_t pidValuesBt;
                        if (PROTOCOL_GetData(msg, (uint8_t*) &pidValuesBt)) { // This will read the data and check the checksum
                            config_t config = CONFIG_GetConfig();
                            config.pid_values.Kp = pidValuesBt.Kp / 1000.0f;
                            config.pid_values.Ki = pidValuesBt.Ki / 100.0f;
                            config.pid_values.Kd = pidValuesBt.Kd / 100000.0f;
                            config.pid_values.integrationLimit = pidValuesBt.integrationLimit / 100.0f;
                            CONFIG_SetConfig(&config);
                            //beepBuzzer(); // Indicate if new values were set
#if DEBUG_PROTOCOL
                            PID_PrintValues("pitch", &config.pid_values); // Print PID Values
#endif
                        }
                    }
                    break;

                case GET_PID:
                    if (msg.length == 0 && PROTOCOL_GetData(msg, NULL)) { // Check length and the checksum
                        msg.length = sizeof(pid_values_bt_t);
                        pid_values_bt_t pidValuesBt;
                        config_t config = CONFIG_GetConfig();
                        pidValuesBt.Kp = config.pid_values.Kp * 1000.0f;
                        pidValuesBt.Ki = config.pid_values.Ki * 100.0f;
                        pidValuesBt.Kd = config.pid_values.Kd * 100000.0f;
                        pidValuesBt.integrationLimit = config.pid_values.integrationLimit * 100.0f;
                        PROTOCOL_SendData(msg, (uint8_t*) &pidValuesBt);
#if DEBUG_PROTOCOL
                        PID_PrintValues("pitch", &config.pid_values); // Print PID Values
#endif
                    }
                    break;
#if 0
                case SET_TARGET:
                    if (msg.length == sizeof(target) && PROTOCOL_GetData(msg, (uint8_t*) &target)) { // This will read the data and check the checksum
                        config_t config = CONFIG_GetConfig();
                        config.targetAngle = (float) target.targetAngle / 100.0;
                        CONFIG_SetConfig(&config);
                        beepBuzzer(); // Indicate if new values were set
#if DEBUG_PROTOCOL
                        Serial.println("SET_TARGET");
#endif
                    }
                    break;

                case GET_TARGET:
                    if (msg.length == 0 && PROTOCOL_GetData(msg, NULL)) { // Check length and the checksum
                        msg.cmd = GET_TARGET;
                        msg.length = sizeof(target);
                        config_t config = CONFIG_GetConfig();
                        target.targetAngle = config.targetAngle * 100.0;
                        PROTOCOL_SendData(msg, (uint8_t*) &target);
#if DEBUG_PROTOCOL
                        Serial.println("GET_TARGET");
#endif
                    }
                    break;
#endif
#if 0
                case SEND_INFO:
                    if (msg.length == sizeof(sendInfo)) { // Make sure that it has the right length
                        if (PROTOCOL_GetData(msg, &sendInfo)) { // This will read the data and check the checksum
#if DEBUG_PROTOCOL
                            Serial.printf("sendInfo: %u\n", sendInfo);
#endif
                        } else
                            sendInfo = 0; // If there was an error, we reset it back to 0, just to be sure
                    }
                    break;
#endif
                case CAL_ACC:
                    if (msg.length == 0 && PROTOCOL_GetData(msg, NULL)) { // Check length and the checksum
                        while (MPU6500_CalibrateAcc(mpu6500)) { // Get accelerometer zero values
                            // Loop until it is successfully calibrated
                        }
                        //beepLongBuzzer();
#if DEBUG_PROTOCOL
                        Serial.println("CAL_ACC");
#endif
                    }
                    break;

                case RESTORE_DEFAULTS:
                    if (msg.length == 0 && PROTOCOL_GetData(msg, NULL)) { // Check length and the checksum
                        CONFIG_SetDefault();
                        //beepLongBuzzer();
#if DEBUG_PROTOCOL
                        Serial.println("RESTORE_DEFAULTS");
#endif
                    }
                    break;

#if DEBUG_PROTOCOL
                default:
                    Serial.printf("Unknown command: %u\n", msg.cmd);
                    break;
#endif
                }
            }
        }
    }
#if 0
    static uint32_t infoTimer = 0;
    if (sendInfo && millis() - infoTimer > 100) {
        infoTimer = millis();
        msg.cmd = SEND_INFO;
        msg.length = sizeof(info);
        info.duty = VESC_GetDuty() * 100.0f;
        info.current = VESC_GetCurrent() * 100.0f;
        info.battery = VESC_GetVoltageIn() * 100.0f;
        PROTOCOL_SendData(msg, (uint8_t*) &info);
    }
#endif
}

// Message protocol (Inspired by MultiWii):
// Request:
// Header: $S>
// cmd: uint8_t
// n length of data: uint8_t
// Data: n uint8_t
// Checksum (calculated from cmd, length and data)

// Response:
// Header: $S<
// cmd: uint8_t
// n length of data: uint8_t
// Data: n uint8_t
// Checksum (calculated from cmd, length and data)
// Carriage return and line feed ("\r\n")

static uint8_t PROTOCOL_GetCheckSum(uint8_t *data, size_t length) {
    if (length == 0)
        return 0;
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++)
        checksum ^= data[i]; // Calculate checksum
    return checksum;
}

static bool PROTOCOL_GetData(msg_t msg, uint8_t *data) {
    if (Serial1.readBytes(data, msg.length) != msg.length) // Read data into buffer
        return false;
    uint8_t checksum;
    if (Serial1.readBytes(&checksum, sizeof(checksum)) != sizeof(checksum)) // Read the checksum
        return false;
    return (PROTOCOL_GetCheckSum((uint8_t*) &msg, sizeof(msg))
            ^ PROTOCOL_GetCheckSum(data, msg.length)) == checksum; // The checksum is calculated from the length, command and the data
}

static void PROTOCOL_SendData(msg_t msg, uint8_t *data) {
    const uint8_t checksum = PROTOCOL_GetCheckSum((uint8_t*) &msg, sizeof(msg)) ^ PROTOCOL_GetCheckSum(data, msg.length);

    Serial1.write(responseHeader);
    Serial1.write((uint8_t*) &msg, sizeof(msg));
    Serial1.write(data, msg.length);
    Serial1.write(checksum); // The checksum is calculated from the length, command and the data
    Serial1.write("\r\n"); // Print carriage return and line feed as well, so it is easy to figure out the line ending in Java
}
