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

#ifndef __SPI_DRIVER_H__
#define __SPI_DRIVER_H__

void SPI_Init(void);

void SPI_Write(uint8_t pin, uint32_t clock, uint8_t regAddr, uint8_t data);

void SPI_Write(uint8_t pin, uint32_t clock, uint8_t regAddr, const uint8_t *data, size_t size);

uint8_t SPI_Read(uint8_t pin, uint32_t clock, uint8_t regAddr);

void SPI_Read(uint8_t pin, uint32_t clock, uint8_t regAddr, uint8_t *recvData, size_t size);

#endif /* __SPI_DRIVER_H__ */
