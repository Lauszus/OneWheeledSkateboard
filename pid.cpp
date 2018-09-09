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

#include "pid.h"

void PID_Init(pid_controller_t *pid, const pid_values_t *pid_values) {
    pid->pid_values = *pid_values;
    pid->low_pass.Fc = pid_values->Fc;
    PID_Reset(pid);
}

static float PID_Update(pid_controller_t *pid, float setPoint, float input, float dInput, bool useDInput, float dt) {
    float error = setPoint - input;

    // P-term
    pid->pTerm = pid->pid_values.Kp * error;

    // I-term
    // Use Trapezoidal Integration, see: http://ecee.colorado.edu/shalom/Emulations.pdf
    pid->iTerm += pid->pid_values.Ki * (error + pid->lastError) / 2.0f * dt; // Multiplication with Ki is done before integration limit, to make it independent from integration limit value
    pid->iTerm = constrain(pid->iTerm, -pid->pid_values.integrationLimit, pid->pid_values.integrationLimit); // Limit the integrated error - prevents windup

    // D-term
    float derivative;
    if (useDInput)
        derivative = -dInput; // Use the D-input directly
    else {
        float deltaError = (error - pid->lastError) / dt; // Calculate difference and compensate for difference in time by dividing by dt
        pid->low_pass.Fc = pid->pid_values.Fc; // Set frequency cutoff in case it has been changed
        derivative = FILTERS_ApplyLowPass(&pid->low_pass, deltaError, dt); // Apply low-pass filter to the derivative term
    }
    pid->lastError = error;
    pid->dTerm = pid->pid_values.Kd * derivative;

    return pid->pTerm + pid->iTerm + pid->dTerm; // Return sum
}

float PID_UpdateClassic(pid_controller_t *pid, float setPoint, float input, float dt) {
    return PID_Update(pid, setPoint, input, 0, false, dt);
}

float PID_UpdateDInput(pid_controller_t *pid, float setPoint, float input, float dInput, float dt) {
    return PID_Update(pid, setPoint, input, dInput, true, dt);
}

void PID_Reset(pid_controller_t *pid) {
    pid->pTerm = pid->iTerm = pid->dTerm = pid->lastError = 0.0f;
    pid->low_pass.prevOutput = 0;
}

void PID_PrintValues(const char *name, const pid_values_t *pid_values) {
    Serial.printf("PID %s: %.4f, %.4f, %.4f, %.4f, %.4f\n", name,
            (double)pid_values->Kp, (double)pid_values->Ki,
            (double)pid_values->Kd, (double)pid_values->integrationLimit,
            (double)pid_values->Fc);
}
