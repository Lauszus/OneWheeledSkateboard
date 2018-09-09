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

#include <SPI.h>

#include "spi_driver.h"

void SPI_Init(void) {
    SPI.begin();
}

static void SPI_Transfer(uint8_t pin, uint32_t clock, uint8_t regAddr, const uint8_t *sendData, uint8_t *recvData, size_t size) {
    SPI.beginTransaction(SPISettings(clock, MSBFIRST, SPI_MODE0));
    digitalWrite(pin, LOW);

    SPI.transfer(regAddr); // Send the register
    SPI.transfer(sendData, recvData, size); // Now send/receive the data

    digitalWrite(pin, HIGH);
    SPI.endTransaction();
}

void SPI_Write(uint8_t pin, uint32_t clock, uint8_t regAddr, uint8_t data) {
    SPI_Transfer(pin, clock, regAddr, &data, NULL, 1);
}

void SPI_Write(uint8_t pin, uint32_t clock, uint8_t regAddr, const uint8_t *data, size_t size) {
    SPI_Transfer(pin, clock, regAddr, data, NULL, size);
}

uint8_t SPI_Read(uint8_t pin, uint32_t clock, uint8_t regAddr) {
    uint8_t data;
    SPI_Transfer(pin, clock, regAddr, NULL, &data, 1);
    return data;
}

void SPI_Read(uint8_t pin, uint32_t clock, uint8_t regAddr, uint8_t *recvData, size_t size) {
    SPI_Transfer(pin, clock, regAddr, NULL, recvData, size);
}
