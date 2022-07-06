/*
 * I2C.h
 *
 *  Created on: Mar 16, 2022
 *      Author: brian
 */

#ifndef I2C_H_
#define I2C_H_

#define NULL 0
#define FUELGAUGE     0x55

#define GPS                 0x42


void I2C_Initialization(I2C_TypeDef * I2Cx);
void I2C_ClearNack(I2C_TypeDef * I2Cx);
void I2C_Start(I2C_TypeDef * I2Cx, uint32_t DevAddress, uint8_t nBytes, uint8_t Direction);
void I2C_Stop(I2C_TypeDef * I2Cx);
void I2C_WaitLineIdle(I2C_TypeDef * I2Cx);
int8_t I2C_Send(I2C_TypeDef * I2Cx, uint8_t SlaveAddress, void *pdata, uint8_t nBytes);
int8_t I2C_Receive(I2C_TypeDef * I2Cx, uint8_t SlaveAddress, uint8_t *pData, uint8_t nBytes);



#endif /* I2C_H_ */
