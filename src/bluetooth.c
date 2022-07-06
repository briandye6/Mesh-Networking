/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f0xx.h"
//#include "RFM69.h"
//#include "RFM69registers.h"
//#include "SPI.h"
//#include <time.h>
//#include <string.h>
#include <stdio.h>

extern uint8_t nodeAddress;

//============================================================================
// Wait for n nanoseconds. (Maximum: 4.294 seconds)
//============================================================================
/*void nano_wait(unsigned int n) {
    asm(    "        mov r0,%0\n"
            "repeat: sub r0,#83\n"
            "        bgt repeat\n" : : "r"(n) : "r0", "cc");
}
*/



#define BufferSize 32
extern uint8_t USART4_Buffer_Rx[71];
uint8_t USART4_Buffer_Tx[71];

void USART(USART_TypeDef *USARTx)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;      // Turn on the clock for Port C

	// Configure PC10 and PC11 for Alternate Function
	GPIOC->MODER &= ~(0x3 << (2*10));
	GPIOC->MODER &= ~(0x3 << (2*11));
	GPIOC->MODER |= 0x2 << (2*10);
	GPIOC->MODER |= 0x2 << (2*11);
	GPIOC->AFR[0] &= ~(0xF << 0);

	GPIOC->OSPEEDR |= (0xF<<(2*10));      // Output Speed: Fast
	GPIOC->PUPDR &= ~(0xF<<(2*10));
	GPIOC->PUPDR |= (0xA<<(2*10));      // Pull Up resistors
	GPIOC->OTYPER &= ~(0x3<<10);    // Output Type: Push-Pull



	USARTx->CR1 &= ~USART_CR1_UE;                         // Disable UART

	RCC->APB1ENR |= RCC_APB1ENR_USART4EN;                 // Turn on the USART4 clock

	USARTx->CR1 &= ~USART_CR1_M;                          // Data Length 8 bits

	USARTx->CR2 &= ~USART_CR2_STOP;                       // 1 stop bit

	USARTx->CR1 &= ~USART_CR1_PCE;                        // No Parity bit

	USARTx->CR1 &= ~USART_CR1_OVER8;                      // Over sample x16


	USARTx->BRR = 0x341;                                 // Baud Rate: 9600

	USARTx->CR1 |= (USART_CR1_TE | USART_CR1_RE);         // Transmit and Receive

	USARTx->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;       // Enable DMA for reception

	USARTx->ICR |= USART_ICR_TCCF;

	//USARTx->CR1 |= USART_CR1_CMIE;
	//USARTx->CR1 |= USART_CR1_RXNEIE;

	USART4->CR1 &= ~USART_CR1_TCIE;
	USARTx->CR1 |= USART_CR1_UE;                          // Enable USART4


	while ((USARTx->ISR & USART_ISR_TEACK) == 0);         // Wait for Transmission ready
	while ((USARTx->ISR & USART_ISR_REACK) == 0);         // Wait for reception ready



	// UART DMA Receive
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;                     // Enable DMA1 clock
	DMA1_Channel6->CCR &= ~DMA_CCR_EN;                    // Disable OMA channel
	DMA1_Channel6->CCR &= ~DMA_CCR_MEM2MEM;               // Disable memory to memory mode
	DMA1_Channel6->CCR &= ~DMA_CCR_PL;                    // Channel priority Level
	DMA1_Channel6->CCR |= DMA_CCR_PL_1;                   // Set OMA priority to high
	DMA1_Channel6->CCR &= ~DMA_CCR_PSIZE;                 // Peripheral data size ee = 8 bits
	DMA1_Channel6->CCR &= ~DMA_CCR_MSIZE;                 // Memory data size: ee = 8 bits
	DMA1_Channel6->CCR &= ~DMA_CCR_PINC;                  // Disable peripheral increment mode
	DMA1_Channel6->CCR |= DMA_CCR_MINC;                   // Enable memory increment mode
	DMA1_Channel6->CCR &= ~DMA_CCR_CIRC;                  // Disable circular mode
	DMA1_Channel6->CCR &= ~DMA_CCR_DIR;                   // Transfer direction: to memory
	DMA1_Channel6->CCR &= ~DMA_CCR_TCIE;                   // Transfer complete interrupt disable
	DMA1_Channel6->CCR |= DMA_CCR_HTIE;                   // Disable Half transfer interrupt
	DMA1_Channel6->CNDTR = 62;                             // Number of data to transfer
	DMA1_Channel6->CPAR = (uint32_t)&(USART4->RDR);       // Peripheral address
	DMA1_Channel6->CMAR = (uint32_t) USART4_Buffer_Rx;    // Receive buffer address

	NVIC_SetPriority(11, 2);
	NVIC_EnableIRQ(11);



	DMA1_Channel6->CCR |= DMA_CCR_EN;                     // Enable DMA1 channel 6


	// UART DMA Transmit
	DMA1_Channel7->CCR &= ~DMA_CCR_MEM2MEM;               //Disable memory to memory mode
	DMA1_Channel7->CCR &= ~DMA_CCR_PL;                    // Clear Channel priority Level
	DMA1_Channel7->CCR |= DMA_CCR_PL_1;                     // Set OMA priority to high
	DMA1_Channel7->CCR &= ~DMA_CCR_PSIZE;                 // Peripheral data size 00 = 8 bits
	DMA1_Channel7->CCR &= ~DMA_CCR_MSIZE;                 // Memory data size: 00 = 8 bits
	DMA1_Channel7->CCR &= ~DMA_CCR_PINC;                  // Disable peripheral increment mode
	DMA1_Channel7->CCR |= DMA_CCR_MINC;                   // Enable memory increment mode
	DMA1_Channel7->CCR &= ~DMA_CCR_CIRC;                  // Disable circular mode
	DMA1_Channel7->CCR |= DMA_CCR_DIR;                    // Transfer direction: to peripheral
	DMA1_Channel6->CCR &= ~DMA_CCR_TCIE;                  // Transfer complete interrupt DISABLE
	DMA1_Channel7->CCR &= ~DMA_CCR_HTIE;                  // Disable Half transfer interrupt
	DMA1_Channel7->CPAR = (uint32_t)&(USART1->TDR);       //Peripheral address


	DMA1_Channel7->CCR &= ~DMA_CCR_EN; // Disable DMA1 channel6





}



void USART_Read (uint8_t *buffer, uint32_t nBytes) {

  for (int i = 0; i < nBytes; i++) {

    while (!(USART4->ISR & USART_ISR_RXNE));

    buffer[i] = USART4->RDR;
  }

}


void USART4_Write (uint8_t *buffer, uint32_t nBytes) {

  for (int i = 0; i < nBytes; i++) {
    while (!(USART4->ISR & USART_ISR_TXE));
    *(uint8_t *)&(USART4->TDR) = buffer[i] & 0xFF;
  }

  while (!(USART4->ISR & USART_ISR_TC));

  USART4->ISR |= USART_ICR_TCCF;

}

void receive( uint8_t *buffer, uint32_t *pCounter) {
  if(USART4->ISR & USART_ISR_RXNE) {          // Check RXNE event
      buffer[*pCounter] = *(uint8_t *)&(USART4->RDR);        // Reading RDR clears the RXNE flag
      (*pCounter)++;                          // Dereference and update memory value
      if((*pCounter) >= BufferSize) {         // Check buffer overflow
          ( *pCounter) = 0;                   // Reset if overflow
      }
  }
}


void DMA_Transmit(uint8_t *pBuffer, uint8_t size) {

  memcpy(USART4_Buffer_Tx, pBuffer, size);
  USART4_Buffer_Tx[size] = (uint8_t) "\n";
  DMA1_Channel7->CCR &= ~DMA_CCR_EN; // Disable DMA1 channel7
  DMA1_Channel7->CNDTR = (uint32_t) size+1;
  DMA1_Channel7->CMAR = (uint32_t) pBuffer;
  DMA1_Channel7->CCR |= DMA_CCR_EN; // Disable DMA1 channel7


}

void HM19_Sleep(void) {
  char command[] = "AT+SLEEP";
  USART4_Write((uint8_t *) command, strlen(command));
}


void HM19_init(void) {
  USART(USART4);                                  // UART enable for HM-19
  //USART4_Write ((uint8_t *) "AT", 2);
  //USART4_Write ((uint8_t *) "AT+RESET", 8);
  //char data[15] = {'A','T','+','N','A','M','E','E','N','I','G','M','A',' ', nodeAddress};
  //USART4_Write ((uint8_t *) data, 8);

}


/*

int main(void)
{
  uint8_t USART4_Buffer_Rx[BufferSize] = {0};
  uint32_t Rx4_Counter = 0, Tx4_Counter = 0;

  USART(USART4, USART4_Buffer_Rx);

  uint8_t * txBuffer = (uint8_t *) "AT";
  USART_Write(USART4, txBuffer, 2);

  uint8_t * tx1Buffer = (uint8_t *) "AT+ROLE0";
	USART_Write(USART4, tx1Buffer, 8);





	  SPI(SPI2);
	  RFM69_reset();
	  if (RFM69_initialize(RF69_915MHZ, 'A', 6) != 1) {    // Node address, network ID
	        return 1;
	  } else {
	      uint8_t VALUE = RFM69_readReg(REG_BITRATEMSB);
	      uint8_t * buffer = "Hello this is a test";
	      for (int i = 0; i< 100; i++){
	          RFM69_send((uint8_t) 'B' , buffer, 20, 0);
	      }
	  }


	return 0;
}

*/
