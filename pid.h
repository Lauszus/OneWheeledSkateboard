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

#ifndef __PID_H__
#define __PID_H__

// TODO: Convert this into a class

#include "filters.h"

/** Struct used to store PID values used by the PID class */
typedef struct {
    float Kp; /*!< Proportional gain */
    float Ki; /*!< Integral gain */
    float Kd; /*!< Derivative gain */
    float integrationLimit; /*!< Integration limit value */
    float Fc; /*!< Cutoff frequency in Hz - this is only used when the "Classic" form is used */
} pid_values_t;

typedef struct {
    pid_values_t pid_values; // Use pointer to pid_values_t struct that are saved in the EEPROM
    low_pass_t low_pass; // Low pass filter used to filter out noise on the derivative term
    float pTerm, iTerm, dTerm, lastError;
} pid_controller_t;

/**
 * The constructor creates the PID controller object and initialize the internal variables.
 * @param  pid          Pointer to a pid_t struct containing used by the PID controller.
 * @param  pid_values   Pointer to a pid_values_t struct containing the PID values.
 */
void PID_Init(pid_controller_t *pid, const pid_values_t *pid_values);

/**
 * Update the PID controller using a pure derivative of the input.
 * @param  setPoint The setpoint/reference for the controller.
 * @param  input    The input to the controller.
 * @param  dt       The time in seconds between last call to the PID controller.
 * @return          Returns the PID controller output.
 */
float PID_UpdateClassic(pid_controller_t *pid, float setPoint, float input, float dt);

/**
 * Update the PID controller using a second input specifying the derivative of the input.
 * @param  setPoint The setpoint/reference for the controller.
 * @param  input    The input to the controller.
 * @param  dInput   The derivative of the input.
 * @param  dt       The time in seconds between last call to the PID controller.
 * @return          Returns the PID controller output.
 */
float PID_UpdateDInput(pid_controller_t *pid, float setPoint, float input, float dInput, float dt);

/** Set all internal states to their default state */
void PID_Reset(pid_controller_t *pid);

void PID_PrintValues(const char *name, const pid_values_t *pid_values);

#endif /* __PID_H__ */
