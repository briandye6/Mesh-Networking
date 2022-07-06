/**
  ******************************************************************************
  * @file    I2C.c
  * @author  Brian Dye
  * @version V1.0
  * @date    16-March-2022
  * @brief   I2C setup, write, read function needed to communicate with Sparkfun
  *          Battery Baby Sitter and SAM-M8Q GPS module
  ******************************************************************************
*/

#include "stm32f0xx.h"
#include "I2C.h"



// I2C Initialization for SAM-M8Q GPS Module
// SDA --> PA10
// SCL --> PA9
// Pull-up resistor 1k ohm
void I2C_Initialization(I2C_TypeDef * I2Cx) {

  // Enable the RCC clock to GPIO Port A and the I2Cx
  // channel, set the MODER fields for PA9 and PA10, and
  // set the alternate function register entries
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
  GPIOA->MODER |= 2<<(2*9) | 2<<(2*10);
  GPIOA->AFR[1] |= 0x440;
  GPIOA->PUPDR |= (GPIO_PuPd_UP << 20) | (GPIO_PuPd_UP << 18);


  I2Cx->CR1 &= ~I2C_CR1_PE;         // Disable to perform reset.
  I2Cx->CR1 &= ~I2C_CR1_ANFOFF;     // 0: Analog noise filter on.
  I2Cx->CR1 &= ~I2C_CR1_ERRIE;      // Error interrupt disable
  I2Cx->CR1 &= ~I2C_CR1_NOSTRETCH;  // Enable clock stretching



  I2C1->TIMINGR = 0;
  I2C1->TIMINGR &= ~I2C_TIMINGR_PRESC;// Clear prescaler
  I2C1->TIMINGR |= 0 << 28; // Set prescaler to 0
  I2C1->TIMINGR |= 3U << 20; // SCLDEL: Data Setup Time
  I2C1->TIMINGR |= 1U << 16; // SDADEL: Data Hold Time
  I2C1->TIMINGR |= 4U << 8; // SCLH
  I2C1->TIMINGR |= 10U << 0; // SCLL






  I2Cx->OAR1 &= ~I2C_OAR1_OA1EN;        // Disable own address 1
  I2Cx->OAR2 &= ~I2C_OAR2_OA2EN;        // Disable own address 2
  I2Cx->CR2 &= ~I2C_CR2_ADD10;          // 0 = 7-bit mode; 1 = 10-bit
  I2Cx->CR2 |= I2C_CR2_AUTOEND;         // Enable the auto end
  I2Cx->CR1 |= I2C_CR1_PE;              // Enable I2Cx
}



int I2C_CheckNack(I2C_TypeDef * I2Cx) {
  int check_NACK = (I2Cx->CR2 & I2C_CR2_NACK) >> 15;
  return check_NACK;
}



void I2C_ClearNack(I2C_TypeDef * I2Cx) {
  I2Cx->CR2 &= ~I2C_CR2_NACK;
}



void I2C_Start(I2C_TypeDef * I2Cx, uint32_t DevAddress, uint8_t nBytes, uint8_t Direction) {
    // Direction: 0 = master requests a write transfer
    // Direction: 1 = master requests a read transfer


    uint32_t tmpreg = I2Cx->CR2;
    tmpreg &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_NBYTES |
                                     I2C_CR2_RELOAD | I2C_CR2_AUTOEND |
                                     I2C_CR2_RD_WRN | I2C_CR2_START |
                                     I2C_CR2_STOP));
    if (Direction == 1)
        tmpreg |= I2C_CR2_RD_WRN; // Read from slave
    else
        tmpreg &= ~I2C_CR2_RD_WRN; // Write to slave

    tmpreg |= (uint32_t)(((uint32_t) (DevAddress<<1) & I2C_CR2_SADD) |
              (((uint32_t) nBytes << 16) & I2C_CR2_NBYTES));

    tmpreg |= I2C_CR2_START;
    I2Cx->CR2 = tmpreg;
}



void I2C_Stop(I2C_TypeDef * I2Cx) {
    // Check if Master has already generated a stop bit
    if (I2Cx->ISR & I2C_ISR_STOPF) { return; }

    // Master: Generate STOP bit after current byte has been transferred.
    I2Cx->CR2 |= I2C_CR2_STOP;

    // Wait until STOPF flag is reset
    while( (I2C1->ISR & I2C_ISR_STOPF) == 0);

    I2C1->ICR |= I2C_ICR_STOPCF; // Write to clear STOPF flag
}


void I2C_WaitLineIdle(I2C_TypeDef * I2Cx) {
  // Wait until bus is ready
  while ( (I2Cx->ISR & I2C_ISR_BUSY) == I2C_ISR_BUSY); // while busy, wait
}



int8_t I2C_Send(I2C_TypeDef * I2Cx, uint8_t SlaveAddress, void *pdata, uint8_t nBytes) {

  if (nBytes <= 0 || pdata == NULL) return -1;

  I2C_WaitLineIdle(I2Cx);

  uint8_t *udata = (uint8_t*)pdata;

  // Last argument is dir: 0 = sending data to the slave.
  I2C_Start(I2Cx, SlaveAddress, nBytes, 0);
  int i;
  for(i = 0; i < nBytes; i++) {
      // TXIS bit is set by hardware when the TXDR register is empty and the
      // data to be transmitted must be written in the TXDR register. It is
      // cleared when the next data to be sent is written in the TXDR reg
      // The TXIS flag is not set when a NACK is received.

      while( (I2Cx->ISR & I2C_ISR_TXIS) == 0);

      // If a nack if received, clearnack, stop I2C, & return
      if (I2C_CheckNack(I2Cx)) { I2C_ClearNack(I2Cx); I2C_Stop(I2Cx); return -1; }


      // TXIS is cleared by writing to the TXDR register.
      I2Cx->TXDR = udata[i] & I2C_TXDR_TXDATA;
  }


  // Wait until TC flag is set or the NACK flag is set.
  while((I2Cx->ISR & I2C_ISR_TC) == 0 && (I2Cx->ISR & I2C_ISR_NACKF) == 0);

  if ( (I2Cx->ISR & I2C_ISR_NACKF) != 0) {
      // Error has occured
      return -1;
  }

  I2C_Stop(I2Cx);

  return 0;
}


int8_t I2C_Receive(I2C_TypeDef * I2Cx, uint8_t SlaveAddress, uint8_t *pData, uint8_t nBytes) {

    if (nBytes <= 0 || pData == NULL) return -1;

    I2C_WaitLineIdle(I2Cx);

    // Last argument is dir: 1 = receiving from the slave
    I2C_Start(I2Cx, SlaveAddress, nBytes, 1);


    int check = 0;
    for (int i = 0; i < nBytes; i++) {
        // Wait until RXNE flag is set
        check = 0;
        while( (I2Cx->ISR & I2C_ISR_RXNE) == 0) {
             check++;
             if (check > 10000) {
                return -1;
             }
        }
        pData[i] = I2Cx->RXDR & I2C_RXDR_RXDATA;
    }

    check = 0;
    while ((I2Cx->ISR & I2C_ISR_TC) == 0) {
        check++;
        if (check > 10000) {
            return -1;
        }
    }

    I2C_Stop(I2Cx);

    return 0;         // indicates success
}
