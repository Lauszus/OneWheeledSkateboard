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

#include <FlexCAN.h> // https://github.com/collin80/FlexCAN_Library

#include "config.h"
#include "imu.h"
#include "mpu6500.h"
#include "pid.h"
#include "pins.h"
#include "protocol.h"
#include "spi_driver.h"
#include "vesc.h"

class CanBusListener: public CANListener {
public:
    bool frameHandler(CAN_message_t &frame, int mailbox, uint8_t controller) {
        if (!VESC_ParseFrame(frame)) {
            // Print the unknown message
            const uint8_t maxIdLength = frame.ext ? 8 : 3;
            Serial.printf("ID: %0*lX, length: %d", maxIdLength, frame.id, frame.len);
            if (frame.len > 0)
                Serial.write(':');
            for (uint8_t i = 0; i < frame.len; i++)
                Serial.printf(" %02X", frame.buf[i]);
            Serial.println();
        }
        return true;
    }
};

static CanBusListener canBusListener;
static mpu6500_t mpu6500;
static pid_controller_t pid_controller;

void setup() {
    Serial.begin(115200);
    //while (!Serial);

    SPI_Init();
    MPU6500_Init(&mpu6500);

    if (!CONFIG_Init()) {
        Serial.write("Calibrating Accelerometer\n");
        while (MPU6500_CalibrateAcc(&mpu6500)) {
            // Loop until calibration is successful
        }
    }

    PROTOCOL_Init();

    // Calibrate gyroscope zero value
    while (MPU6500_CalibrateGyro()); // Run again if it is moved while calibrating

    // Initialize CAN-Bus interfaces at 500 kHz
    Can0.begin(5e5);

    // Split the number of mailboxes 50/50
    Can0.setNumTXBoxes(Can0.getNumMailBoxes() / 2);

    Can0.attachObj(&canBusListener); // Attach the listener
    canBusListener.attachGeneralHandler(); // Configure the listener to receive on all mailboxes

    // Receive messages with an extended ID
    CAN_filter_t filter;
    filter.id = 0;
    filter.ext = 1;
    filter.rtr = 0;
    for (uint8_t i = 0; i < Can0.getNumRxBoxes(); i++)
        Can0.setFilter(filter, i);

    pinMode(DEADMAN_PIN, INPUT_PULLUP);

    config_t config = CONFIG_GetConfig();
    PID_Init(&pid_controller, &config.pid_values);

    //BUZZER_Init();
    //beepBuzzer(); // Indicate startup

    Serial.println("Started");
}

static low_pass_t motor_lpf = { .Fc = 10.0f, .prevOutput = 0 };

void loop() {
    angle_t angle;
    if (IMU_Task(&mpu6500, &angle)) {
#if 1
        // If the skateboard is laying down, it has to be put in a horizontal position before it starts balancing
        // If it's already balancing it has to be +-10 degrees before it stops trying to balance
        static bool layingDown = true;
        config_t config = CONFIG_GetConfig();
        if (digitalRead(DEADMAN_PIN)
                || (layingDown && fabsf(angle.pitch - config.targetAngle) > 2.0f * DEG_TO_RAD)
                || (!layingDown && fabsf(angle.pitch - config.targetAngle) > 12.0f * DEG_TO_RAD)) {
            layingDown = true; // The skateboard is in a unsolvable position, so turn off the motor and wait until it's horizontal again
            PID_Reset(&pid_controller);
            VESC_SetCurrent(0);
        } else {
            pid_controller.pid_values = config.pid_values; // Set the PID values in case they have been changed - TODO: Use pointer
            pid_controller.pid_values.Kp *= 10.0f;
            pid_controller.pid_values.Ki *= 100.0f;
            pid_controller.pid_values.Kd *= 1000.0f;

            layingDown = false; // It's no longer laying down

            // Inputs in deg and deg/s - output in amps
            float vesc_current_out = -PID_UpdateDInput(&pid_controller, config.targetAngle, angle.pitch * RAD_TO_DEG, mpu6500.gyroRateLpf.pitch * RAD_TO_DEG, MPU6500_INT_DT);
            vesc_current_out = FILTERS_ApplyLowPass(&motor_lpf, vesc_current_out, MPU6500_INT_DT);
            static uint8_t out_counter = 0;
            if (++out_counter >= MPU6500_INT_FREQ_HZ / 100) { // Run at 100 Hz
                //Serial1.printf("%f,%f,%f\n", pid_controller.pTerm, pid_controller.iTerm, pid_controller.dTerm);
                out_counter = 0;
                //VESC_SetCurrent(vesc_current_out);
                VESC_SetDuty(vesc_current_out);
                //Serial.println(vesc_current_out);
            }
        }
#else
        Serial.println(angle.pitch * RAD_TO_DEG);
        //Serial.println(!digitalRead(DEADMAN_PIN) ? "ON" : "OFF");
        /*if (!digitalRead(DEADMAN_PIN))
            VESC_SetDuty(constrain(angle.pitch * RAD_TO_DEG, -5, 5));*/
#endif
    }

    // Parse data sent from the Android application
    PROTOCOL_ParseData(&mpu6500, &angle);
}
