/**
  ******************************************************************************
  * @file    main.c
  * @author  Brian Dye
  * @version V1.0
  * @date    01-January-2022
  * @brief   STP Switch Initialization, Serial Communication enable,
             Setup and Enable Interrupts, Interrupt Service Routines,
             GPS and Battery Moniter Helper Functions
  ******************************************************************************
*/


#include "stm32f0xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <lcd.h>
#include <I2C.h>
#include <bluetooth.h>
#include <RFM69.h>
#include <RFM69registers.h>
#include <SPI.h>
#include <STPswitch.h>




STPswitch * switchNode;
uint8_t USART4_Buffer_Rx[71];
char latitude[20] = {0};
char longitude[20] = {0};
uint8_t soc;
uint8_t nodeAddress;


uint16_t StateOfCharge(void) {
  // Send two byte command for state of charge
  uint8_t command[2] = {0x1c, 0x1d};
  if (I2C_Send(I2C1, FUELGAUGE, &(command[0]), 1) == -1) {
      return 0;
  }
  //I2C_Send(I2C1, FUELGAUGE, &(command[1]), 1);


  // Read two byte from fuel gauge
  uint8_t data[2] = {0};                            // State of Charge (SOC)
  I2C_Receive(I2C1, FUELGAUGE, data, 2);

  // Return State of Charge
  return ((uint16_t) data[1] << 8) | data[0];

}



uint8_t get_coord(char * lat, char * lon) {
      uint8_t data[35] = {0};
      char* flag;
      char* temp;
      char* degree;
      char* orient;
      size_t len;
      uint8_t DATA_STREAM = 0xFF;

      if (I2C_Send(I2C1, 0x42, &DATA_STREAM, 1) != 0) {return 0;}
      if (I2C_Receive(I2C1, 0x42, data, 20) != 0) {return 0;}

      flag = strtok((char*)data, ",");
      // TODO: parse the data
      if(*flag == 36){  // 36 --> "$"
          flag = strtok(NULL, ",");
          flag = strtok(NULL, ",");
          if(*flag == 65){  // 65 --> "A"
              // move the rest of the string to the string lat and lon
              degree = strtok(NULL, ",");
              orient = strtok(NULL, ",");
              strcat(degree, orient);
              len = strlen(degree);
              strcpy(lon, degree);
              strcat(lon, "\n");
              degree = strtok(NULL, ",");
              orient = strtok(NULL, ",");
              strcat(degree, orient);
              len = strlen(degree);
              strcpy(lat, degree);
              strcat(lat, "\n");
              return 1;
          }
      }
      return 0;
  }





// Periodically poll battery and GPS coordinates
// Send a control packet
////////////////////////////////////
void TIM6_DAC_IRQHandler(void)
{

    uint8_t value = StateOfCharge();
    if (value > 0 && value <= 100) {
        soc = value;
    }
    get_coord( latitude, longitude);

    sendControlPacket(switchNode);

    // Reset Timer 6 seconds
    TIM6->SR = 0;

}


void setup_tim6(void)
{
  RCC->APB1ENR |= (1<<4);           // Enable timer 6 clock
  TIM6->PSC = 11999;                  // Set precale value
  TIM6->ARR = 3999;                   // ARR Value
  TIM6->CR1 &= ~TIM_CR1_DIR;

  TIM6->DIER |= TIM_DIER_UIE;       // Set the UIE bit in the DIER register
  TIM6->CR1 |= TIM_CR1_CEN;         // Set CEN bit of TIM6 CR1
  //NVIC->ISER[0] = (1<<TIM6_DAC_IRQn);  // Update NVIC ISER for timer 6
  NVIC_SetPriority(TIM6_DAC_IRQn, 3);
  NVIC_EnableIRQ(TIM6_DAC_IRQn);
}



void EXTI4_15_IRQHandler(void) {
  // -------------------------------------------
  // LCD interrupt handler
  // -------------------------------------------
  if ((EXTI->PR & EXTI_PR_PR6) == EXTI_PR_PR6) {
                                           // Setup SP1 for LCD
      // Initialize the LCD
      LCD_Init();
      LCD_Clear(WHITE);
      LCD_Circle(230, 275, 8, 0, BLACK);
      LCD_DrawFillRectangle(220, 252, 229, 297,  WHITE);

      // Draw Battery Outline
      LCD_DrawRectangle(10, 300, 230, 250, BLACK);


      // Get Battery Level and Display it
      float level = 212.0 * soc / 100.0 + 15;
      LCD_DrawFillRectangle( 14, 254, (u16) level, 295,  0x07ED);



      // Network Status
      char * p = "Status:";
      LCD_DrawString(90, 20, BLACK, WHITE, p, 20, 0);
      LCD_DrawFillRectangle( 0, 35, 240, 45,  WHITE);
      if (switchNode->connected) {
          char * q = "Connected to Network";
          LCD_DrawString(15, 50, 0x07E8, BLACK, q, 22, 0);
          LCD_DrawFillRectangle( 15, 67, 240, 95,  WHITE);
      } else {
          char * q = "Not Connected";
          LCD_DrawString(50, 50, RED, BLACK, q, 25, 0);
          LCD_DrawFillRectangle( 15, 67, 240, 95,  WHITE);
      }


      // GPS Location:
      char * r = "Current Location:";
      LCD_DrawString(30, 90, BLACK, WHITE, r, 22, 0);
      LCD_DrawFillRectangle( 30, 107, 240, 120,  WHITE);


      // Valid Cordinates Received
      LCD_DrawString(60, 115, BLUE, BLACK, latitude, 22, 0);
      LCD_DrawFillRectangle( 60, 130, 169, 150,  BLACK);

      LCD_DrawString(60, 135, RED, BLACK, longitude, 22, 0);
      LCD_DrawFillRectangle( 60, 152, 169, 180,  WHITE);


      EXTI->PR |= EXTI_PR_PR6;


  // ----------------------------------------------------
  // RFM69 Packet received interrupt handler
  // ----------------------------------------------------
  } else if ((EXTI->PR & EXTI_PR_PR10) == EXTI_PR_PR10) {
      uint8_t packet[74] = {0};
      uint8_t packetSize = RFM69_handleReceivedPacket(packet);

      if (packetSize > 0) {
          // Packet needs to be ROUTED
          // Clear interrupt
          EXTI->PR |= EXTI_PR_PR10;
          handlePacket(switchNode, packet, packetSize);
      } else {
          // False Alarm
          // Clear interrupt
          EXTI->PR |= EXTI_PR_PR10;
      }



  }

}



void DMA1_CH4_5_6_7_DMA2_CH3_4_5_IRQHandler(void) {
  if ( ((DMA1->ISR & DMA_ISR_HTIF6) == DMA_ISR_HTIF6)) {

      //uint8_t value = USART4->RDR; // Reading RDR clears the RXNE flag
      // Read Command
      char command[8] = {0};
      nano_wait(10000000);
      memcpy(command, (char *) USART4_Buffer_Rx, 8);


      // From command take action
      if (strncmp(command, "get+ft", 6) == 0) {
          USART4_Write ((uint8_t *) "Forwarding Table:\nA\nB\nC\n", 24);
          ListNode * p = switchNode->forwarding_table->head;
          char data[2] = {'\n'};
          while (p != NULL) {
              data[0] = p->key;
              USART4_Write ((uint8_t *) data, 2);
              p = p->next;
          }
          USART4_Write ((uint8_t *) "\n", 1);


      } else if (strncmp(command, "get+gps", 7) == 0) {
          if (latitude[0] != 0 ) {

              // Send Latitude
              USART4_Write ((uint8_t *) latitude, 11);

              //DMA_Transmit(latitude, 11);

              // Send Longitude
              USART4_Write ((uint8_t *) longitude, 11);

          //DMA_Transmit(longitude, 11);
          } else {
              USART4_Write ((uint8_t *) "No Coordinates Available\n", 25);
              //DMA_Transmit((uint8_t *) "Error: No Cordinates", 20);
          }

      } else if (strncmp(command, "get+bat", 7) == 0) {
          char data[3] = {'\n'};
          sprintf(data, "%2d", soc);
          data[2] = '\n';
          USART4_Write ((uint8_t *) data, 3);
          //DMA_Transmit((uint8_t *) data, 3);



        } else if (strncmp(command, "UDP+Pack", 8) == 0) {
              uint8_t * START = &(USART4_Buffer_Rx[10]);
              uint8_t nn[2] = {0};
              nano_wait(10000000);
              memcpy(nn, &(USART4_Buffer_Rx[8]), 2);
              uint8_t packetSize = atoi((const char *) nn);
              uint8_t packet[74] = {0};
              nano_wait(10000000);
              memcpy(&(packet[2]), START, packetSize);
              packet[5] = packetSize;
              // handle received packet from data link layer
              // RFM69_sendPacket((uint8_t) 'B', DATA_PACKET, packet, strlen(packet));
              //RFM69_sendPacket((uint8_t) packet[1], DATA_PACKET, packet, packetSize);
              handlePacket(switchNode, packet, packetSize);


        } else if (strcmp(command, "reset") == 0) {
              return;

        } else if (command[0] == 'O' && command[1] == 'K') {
            // Bluetooth module has status update



        } else {
            //USART4_Write ((uint8_t *) "Error: Incorrect Command\n", 24);
            //DMA_Transmit((uint8_t *) "Error: Incorrect Command\n", 24);
        }


        // Write 1 to clear the corresponding TCIF flag
        DMA1->IFCR |= DMA_IFCR_CHTIF6;
        DMA1_Channel6->CCR &= ~DMA_CCR_EN;                     // Disable DMA channel
        DMA1_Channel6->CNDTR = 62;
        DMA1_Channel6->CCR |= DMA_CCR_EN;                     // Enable DMA channel


  }


}


void SYSTEM_SLEEP(void) {
  NVIC_DisableIRQ(TIM6_DAC_IRQn);
  NVIC_DisableIRQ(7);
  NVIC_DisableIRQ(USART3_8_IRQn);
  RFM69_Sleep();
  HM19_Sleep();
  DMA1_Channel7->CCR &= ~DMA_CCR_EN;              // Disable DMA1 channel7
  DMA1_Channel6->CCR &= ~DMA_CCR_EN;              // Disable DMA1 channel6
  USART4->CR1 &= ~USART_CR1_UE;                   // Disable USART4
  SPI1->CR1 &= ~SPI_CR1_SPE;                      // Disable SPI1
  SPI2->CR1 &= ~SPI_CR1_SPE;                      // Disable SPI2
  I2C1->CR1 &= I2C_CR1_PE;                        // Disable I2C1
  while (1) {
      __WFI();
  }


}

void RFM69_TESTsender() {
  uint8_t packet[9] = {nodeAddress, RF69_BROADCAST_ADDR, 0, 9, 'H', 'e', 'l','l', 'o'};
  while (1) {
      RFM69_sendPacket(RF69_BROADCAST_ADDR, DATA_PACKET, packet, 9);
      nano_wait(200000000);
  }
}


void RFM69_TESTreceiver(void) {
  RFM69_setMode(RF69_MODE_RX);
  while (1) {
      __WFI();
  }
}

int main(void) {
  nodeAddress = (uint8_t) 'A';
  switchNode = createSTPswitch(nodeAddress);      // Initialize the Switch
  SPI(SPI1);
  LCD_EXTI6_Initialize();                         // LCD Interrupt Enable
  I2C_Initialization(I2C1);                       // GPS and Fuel Gauge

  //HM19_init();
  //HM19_Sleep();

  RFM69_reset();
  if (RFM69_initialize(nodeAddress, 6) != 1) {     // Node address, network ID
      return 1;
  } else {
      USART(USART4);
      setup_tim6();



  }

}


  //SPI(SPI1);
  //I2C_Initialization(I2C1);
  //setup_tim6();
  //USART(USART4);
  //char str[] = "Hello Brian";
  //USART_Write(USART4, (uint8_t *) str, strlen(str));







    /*
    uint8_t USART4_Buffer_Rx[BufferSize] = {0};
    uint32_t Rx4_Counter = 0, Tx4_Counter = 0;

    USART(USART4, USART4_Buffer_Rx);                // Setup UART for Bluetooth
    SPI(SPI2);                                      // Setup SPI2 for RFM69
    SPI(SPI1);                                      // Setup SPI1 for LCD


    //uint8_t * txBuffer = (uint8_t *) "AT";
    //USART_Write(USART4, txBuffer, 2);

    //uint8_t * tx1Buffer = (uint8_t *) "AT+ROLE0";
    //USART_Write(USART4, tx1Buffer, 8);

    // Send a Packet using RFM69
    RFM69_reset();
    if (RFM69_initialize(RF69_915MHZ, 'A', 6) != 1) {    // Node address, network ID
          return 1;
    } else {
        uint8_t VALUE = RFM69_readReg(REG_BITRATEMSB);
        uint8_t * buffer = "Hello this is a test";
        for (int i = 0; i< 100; i++){
            RFM69_send((uint8_t) 'B' , USART4_Buffer_Rx, 20, 0);
        }
    }
    LCD_Init();
    LCD_Clear(BLACK);


    return 0;
    */



