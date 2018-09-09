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

#include "vesc.h"

#define VESC_CAN_ID             (0U) /*!< VESC CAN-Bus ID */
#define VESC_POLE_PAIRS         (4U)

#define VESC_MAXIMUM_CURRENT    (20.0f) // TODO: Increase this
#define VESC_MAXIMUM_DUTY       (95.0f)

#define VESC_ID_MASK            (0x00FF) /*!< Mask used for extracting the VESC ID from the CANBus extended ID */
#define VESC_ID_SHIFT           (0U)
#define VESC_PACKET_MASK        (0xFF00) /*!< Mask used for extracting the VESC packet ID from the CANBus extended ID */
#define VESC_PACKET_SHIFT       (8U)

static CAN_message_t tx_msg;

// These are updated in the CAN-Bus interrupt, so they are declared volatile
static volatile float duty_cycle, vesc_voltage_in, vesc_current_in;

/** VESC command packet ID's */
typedef enum {
    VESC_CAN_PACKET_SET_DUTY = 0, /*!< Set the VESC duty cycle */
    VESC_CAN_PACKET_SET_CURRENT, /*!< Set the VESC current */
    VESC_CAN_PACKET_SET_CURRENT_BRAKE, /*!< Set the VESC brake current */
    VESC_CAN_PACKET_SET_RPM, /*!< Set the VESC RPM */
    VESC_CAN_PACKET_SET_POS, /*!< Set the VESC position */
    VESC_CAN_PACKET_FILL_RX_BUFFER, /*!< Used to fill the RX buffer in the VESC. The offset is given in the first byte */
    VESC_CAN_PACKET_FILL_RX_BUFFER_LONG, /*!< Used to fill the RX buffer in the VESC. The offset is given in the two bytes */
    VESC_CAN_PACKET_PROCESS_RX_BUFFER, /*!< Process the data in the RX buffer */
    VESC_CAN_PACKET_PROCESS_SHORT_BUFFER, /*!< Process the received data */
    VESC_CAN_PACKET_STATUS, /*!< Primary status message sent from the VESC */
    VESC_CAN_PACKET_SET_CURRENT_REL, /*!< Set the current relative to minimum and maximum current limits */
    VESC_CAN_PACKET_SET_CURRENT_BRAKE_REL, /*!< Set the braking current relative to minimum and maximum current limits */
    CAN_PACKET_SET_CURRENT_HANDBRAKE,
    CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
    VESC_CAN_PACKET_STATUS_2, /*!< Custom secondary status message sent from the VESC */
} VESC_CAN_PACKET_ID_e;

void VESC_SetDuty(float duty_cycle) {
    duty_cycle = constrain(duty_cycle, -100.0f, 100.0f);

    tx_msg.id = VESC_CAN_PACKET_SET_DUTY << 8 | VESC_CAN_ID;
    tx_msg.ext = 1;
    tx_msg.len = 4;

    int32_t duty_cycle_raw = (int32_t)roundf(duty_cycle * 1000.0f); // Convert into raw units for the VESC

    tx_msg.buf[0] = (duty_cycle_raw >> 24) & 0xFF;
    tx_msg.buf[1] = (duty_cycle_raw >> 16) & 0xFF;
    tx_msg.buf[2] = (duty_cycle_raw >> 8) & 0xFF;
    tx_msg.buf[3] = duty_cycle_raw & 0xFF;

    if (!Can0.write(tx_msg)) {
        //Serial.println("VESC_SetDuty failed");
    }
}

void VESC_SetCurrent(float current) {
    current = constrain(current, -VESC_MAXIMUM_CURRENT, VESC_MAXIMUM_CURRENT); // Make sure the current is within the limits of the system

    tx_msg.id = VESC_CAN_PACKET_SET_CURRENT << 8 | VESC_CAN_ID;
    tx_msg.ext = 1;
    tx_msg.len = 4;

    int32_t current_raw = (int32_t)roundf(current * 1000.0f); // Convert into raw units for the VESC

    tx_msg.buf[0] = (current_raw >> 24) & 0xFF;
    tx_msg.buf[1] = (current_raw >> 16) & 0xFF;
    tx_msg.buf[2] = (current_raw >> 8) & 0xFF;
    tx_msg.buf[3] = current_raw & 0xFF;

    if (!Can0.write(tx_msg)) {
        //Serial.println("VESC_SetCurrent failed");
    }
}

float VESC_GetDuty(void) {
    return duty_cycle;
}

float VESC_GetCurrent(void) {
    return vesc_current_in;
}

float VESC_GetVoltageIn(void) {
    return vesc_voltage_in;
}

bool VESC_ParseFrame(CAN_message_t &rx_msg) {
    bool parsed = false;
    if ((rx_msg.id & VESC_ID_MASK) >> VESC_ID_SHIFT == VESC_CAN_ID) {
        VESC_CAN_PACKET_ID_e packet_id = (VESC_CAN_PACKET_ID_e)((rx_msg.id & VESC_PACKET_MASK) >> VESC_PACKET_SHIFT);

        if (packet_id == VESC_CAN_PACKET_STATUS) {
            parsed = true;
            duty_cycle = (float)((rx_msg.buf[6] << 8) | rx_msg.buf[7]) / 10.0f;
#if 0
            int32_t erpm = (int32_t)((rx_msg.buf[0] << 24) | (rx_msg.buf[1] << 16) | (rx_msg.buf[2] << 8) | rx_msg.buf[3]);
            float rpm = (float)erpm / (float)VESC_POLE_PAIRS;
            Serial.printf("VESC RPM: %.1f, total current: %.1f A, duty cycle: %.1f %%\n",
                    (double)rpm,
                    (double)((int16_t)((rx_msg.buf[4] << 8) | rx_msg.buf[5]) / 10.0f),
                    (double)duty_cycle);
#endif
        } else if (packet_id == VESC_CAN_PACKET_STATUS_2) {
            parsed = true;
            vesc_voltage_in = (float)((rx_msg.buf[0] << 8) | rx_msg.buf[1]) / 100.0f;
            vesc_current_in = (float)((rx_msg.buf[2] << 8) | rx_msg.buf[3]) / 10.0f;
#if 0
            float vesc_power_in = vesc_voltage_in * vesc_current_in;
            Serial.printf("VESC voltage: %.2f V, current in: %.1f V, power: %.2f, mosfet temp: %.2f C, motor temp: %.2f C\n",
                    (double)vesc_voltage_in,
                    (double)vesc_current_in,
                    (double)vesc_power_in,
                    (double)((int16_t)((rx_msg.buf[4] << 8) | rx_msg.buf[5]) / 100.0f),
                    (double)((int16_t)((rx_msg.buf[6] << 8) | rx_msg.buf[7]) / 100.0f));
#endif
        } else
            Serial.printf("Unhandled VESC command: %d\n", packet_id);
    }
    return parsed;
}
