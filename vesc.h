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

#ifndef __VESC_H__
#define __VESC_H__

#include <FlexCAN.h> // https://github.com/collin80/FlexCAN_Library

void VESC_SetDuty(float duty_cycle);

void VESC_SetCurrent(float current);

bool VESC_ParseFrame(CAN_message_t &rx_msg);

float VESC_GetDuty(void);

float VESC_GetCurrent(void);

float VESC_GetVoltageIn(void);

#endif /* __VESC_H__ */
