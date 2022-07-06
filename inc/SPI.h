/*
 * SPI.h
 *
 *  Created on: Feb 23, 2022
 *      Author: brian
 */

#ifndef SPI_H_
#define SPI_H_
#include "stm32f0xx.h"

uint8_t SPI(SPI_TypeDef * SPIx);
void SPI_Write(SPI_TypeDef * SPIx, const uint8_t *txBuffer, uint8_t * rxBuffer, int size);
void SPI_Read(SPI_TypeDef * SPIx, uint8_t *rxBuffer, int size);
void nano_wait(unsigned int n);


#endif /* SPI_H_ */
