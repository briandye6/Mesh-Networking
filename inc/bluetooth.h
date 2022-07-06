/*
 * bluetooth.h
 *
 *  Created on: Mar 8, 2022
 *      Author: brian
 */

#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_
#include "stm32f0xx.h"
void USART(USART_TypeDef * USARTx);
void USART4_Read (uint8_t *buffer, uint32_t nBytes);
void USART4_Write (uint8_t *buffer, uint32_t nBytes);
void receive(uint8_t *buffer, uint32_t *pCounter);
void DMA_Transmit(uint8_t *pBuffer, uint8_t size);
HM19_Sleep(void);
HM19_init(void);

#endif /* BLUETOOTH_H_ */
