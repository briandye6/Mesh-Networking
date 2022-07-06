


#include "stm32f0xx.h"


//============================================================================
// Wait for n nanoseconds. (Maximum: 4.294 seconds)
//============================================================================
/*
void nano_wait(unsigned int n) {
    asm(    "        mov r0,%0\n"
            "repeat: sub r0,#83\n"
            "        bgt repeat\n" : : "r"(n) : "r0", "cc");
}
*/


void SPI(SPI_TypeDef * SPIx) {

  if(SPIx == SPI2) {
      // Turn on RCC clock for GPIO ports B
      RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

      // Configure pins PB12, 13, 14, 15 to be AF (SPI)
      // Configure pin PB11 to be output          (RFM reset control)
      // Configure pin PB10, 9, 5 to be input     (DIO pin Read)
      GPIOB->MODER &= ~0xFFC00000;
      GPIOB->MODER |= 0xA9000000;

      // Configure PB10 PB9 PB5 with PULL down resistor   (Must be driven high)
      GPIOB->PUPDR |= (GPIO_PuPd_DOWN << 20) | (GPIO_PuPd_DOWN << 18) | (GPIO_PuPd_DOWN << 10);



      // Enable the clock to SPI2 in RCC_APB1ENR.
      RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
      SPIx->CR1 &= ~SPI_CR1_SPE;        // Disable SPI
      SPIx->CR1 &= ~SPI_CR1_RXONLY;     //  Transmit and Receive Mode
      SPIx->CR1 &= ~SPI_CR1_BIDIMODE;   // 2-Line Uindirectional data mode
      SPIx->CR1 &= ~SPI_CR1_BIDIOE;     // Turn off: Only used in BIDIR mode
      SPIx->CR2 = SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0; // 8 Bit mode
      SPIx->CR1 &= ~SPI_CR1_LSBFIRST;   // MSB First
      SPIx->CR1 &= ~SPI_CR1_CPHA;       // First clock transition for first data capture
      SPIx->CR1 &= ~SPI_CR1_CPOL;       // Clock polarity low when idle
      SPIx->CR1 |= SPI_CR1_BR_0;        // Set clock to 8 MHz / 8 = 1 MHz
      SPIx->CR2 &= ~SPI_CR2_FRF;        // Enable Motorola Mode
      SPIx->CR1 |= SPI_CR1_SSM;         // Software Slave Select
      SPIx->CR1 |= SPI_CR1_MSTR;        // Master Mode Enable
      //SPIx->CR1 &= ~SPI_CR1_BR;         // Clear baud rate
      SPIx->CR1 |= SPI_CR1_SSI;         // Manage NSS using Software
      SPIx->CR2 |= SPI_CR2_NSSP;        // Enable NSSP Pulse Management
      //SPIx->CR2 |= SPI_CR2_SSOE;        // SS Output enable
      SPIx->CR2 |= SPI_CR2_FRXTH;       // RXNE goes high and stays high until
                                        // RXFIFO is >= 1/4 (8 Bit)
      RFM69_SetCSPin(1);
      // Enable SPI2
      SPIx->CR1 |= SPI_CR1_SPE;


  } else if (SPIx == SPI1) {
      RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

      GPIOA->MODER &= ~0xCFF0;
      GPIOA->MODER |= 0x8A00 | 0x50;
      RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
      // Configure for master mode
      SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE;
      // Set Baud rate as high as possible
      SPI1->CR1 &= ~0x38;
      // Set SSOE/NSSP as you did for SPI2, but leave
      // the word size set to 8-bit (the default).
      SPI1->CR2 &= ~(SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_DS_3);
      SPI1->CR2 |= SPI_CR2_SSOE | SPI_CR2_NSSP | SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2;
      // Enable SPI1
      SPI1->CR1 |= SPI_CR1_SPE;

  }
}



void SPI_Write(SPI_TypeDef * SPIx, const uint8_t *txBuffer, uint8_t * rxBuffer, int size) {

  for (int i = 0; i < size; i++) {
      // Wait for TXE (Transmit buffer empty)
      while( (SPIx->SR & SPI_SR_TXE ) != SPI_SR_TXE );
      //SPIx->DR = txBuffer[i];
      *(uint8_t *)&(SPIx->DR) = txBuffer[i];

      // Wait for RXNE (Receive buffer not empty)
      while( (SPIx->SR & SPI_SR_RXNE ) != SPI_SR_RXNE );
      rxBuffer[i] = *(uint8_t *)&(SPIx->DR);

  }

  // Wait for BSY flag cleared
  while( (SPIx->SR & SPI_SR_BSY) == SPI_SR_BSY );



}

void SPI_Read(SPI_TypeDef * SPIx, uint8_t *rxBuffer, int size) {

  for (int i = 0; i < size; i++) {

      // Wait for TXE (Transmit buffer empty)
      while( (SPIx->SR & SPI_SR_TXE ) != SPI_SR_TXE );
      // The clock is controlled by master.
      // Thus, the master must send a byte
      *(uint8_t *)&SPIx->DR = 0x00; // A dummy byte

      // data to the slave to start the clock.
      while( (SPIx->SR & SPI_SR_RXNE ) != SPI_SR_RXNE );
      rxBuffer[i] = *(uint8_t *)&(SPIx->DR);
  }

  // Wait for BSY flag cleared
  while( (SPIx->SR & SPI_SR_BSY) == SPI_SR_BSY );

}

/*
// function to transfer 1byte on SPI with readback
uint8_t SPI_transfer8(uint8_t value) {

  uint8_t valueRead = 0;
  SPI_Write(SPI2, &value, &valueRead, 1);
  return valueRead;

}
*/
