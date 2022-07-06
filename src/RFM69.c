#include "RFM69.h"
#include "RFM69registers.h"
#include "SPI.h"
#include "stm32f0xx.h"
#include <time.h>





static volatile uint8_t nodeAddress;
static uint8_t _powerLevel = 31;


void RFM69_interruptOFF(void) {
  NVIC_DisableIRQ(USART3_8_IRQn);
  NVIC_DisableIRQ(TIM6_DAC_IRQn);
  EXTI->IMR &= ~EXTI_IMR_MR10;
  NVIC_DisableIRQ(7);
}

void RFM69_interruptON(void) {
  EXTI->IMR |= EXTI_IMR_MR10;
  NVIC_EnableIRQ(USART3_8_IRQn);
  NVIC_EnableIRQ(7);
  NVIC_EnableIRQ(TIM6_DAC_IRQn);
}


// function to control the GPIO tied to RFM69 chip select (parameter HIGH or LOW)
void RFM69_SetCSPin(uint8_t value) {
  if (value == 1) {
        GPIOB->ODR |= 0x1000;
    } else {
        GPIOB->ODR &= ~0x1000;
    }
}

void RFM69_select(void) {
  //interruptOFF();
  RFM69_SetCSPin(0);
}

void RFM69_unselect(void) {
  RFM69_SetCSPin(1);
  //interruptON();
}

// function to read GPIO connected to RFM69 DIO0 (RFM69 interrupt signalling)
uint16_t RFM69_ReadDIOPin(uint8_t pin) {
  switch (pin){
        case 0:
          return GPIOB->IDR & (1<<10);
          break;
        case 2:
          return GPIOB->IDR & (1<<9);
          break;
        case 5:
          return GPIOB->IDR & (1<<5);
          break;
        default:
          return 0;
  }
}

uint16_t RFM69_ModeReady(void) {
  return RFM69_ReadDIOPin(5);
}



// function to transfer 1byte on SPI with readback
uint8_t SPI_transfer8(uint8_t value) {

  uint8_t valueRead = 0;
  //*(uint8_t *)&(SPI2->DR) = value;
  SPI_Write(SPI2, &value, &valueRead, 1);
  return valueRead;

}

uint8_t RFM69_readReg(uint8_t addr)
{
  uint8_t regval;
  RFM69_select();
  SPI_transfer8(addr & 0x7F);
  regval = SPI_transfer8(0);
  RFM69_unselect();
  return regval;
}

void RFM69_writeReg(uint8_t addr, uint8_t value)
{
  RFM69_select();
  SPI_transfer8(addr | 0x80);
  SPI_transfer8(value);
  RFM69_unselect();
}








// return the frequency (in Hz)
uint32_t RFM69_getFrequency()
{
  return RF69_FSTEP * (((uint32_t) RFM69_readReg(REG_FRFMSB) << 16) + ((uint16_t) RFM69_readReg(REG_FRFMID) << 8) + RFM69_readReg(REG_FRFLSB));
}



void RFM69_setMode(uint8_t newMode)
{

  switch (newMode) {
    case RF69_MODE_TX:
      // Turn on Transmitter
      RFM69_writeReg(REG_OPMODE, (RFM69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
      if (ISRFM69HW) RFM69_setHighPowerRegs(1);
      break;
    case RF69_MODE_RX:
      // Turn on the receiver
      // set DIO0 to "PAYLOADREADY" in receive mode
      RFM69_writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01);

      RFM69_writeReg(REG_OPMODE, (RFM69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
      if (ISRFM69HW) RFM69_setHighPowerRegs(0);
      break;
    case RF69_MODE_SYNTH:
      RFM69_writeReg(REG_OPMODE, (RFM69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
      break;
    case RF69_MODE_STANDBY:
      RFM69_writeReg(REG_OPMODE, (RFM69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
      break;
    case RF69_MODE_SLEEP:
      RFM69_writeReg(REG_OPMODE, (RFM69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
      break;
    default:
      return;

  }


  // Wait for ModeReady
  while (!RFM69_ModeReady());


}

//put transceiver in sleep mode to save battery - to wake or resume receiving just call RFM69_receiveDone()
void RFM69_Sleep(void) {
  RFM69_setMode(RF69_MODE_SLEEP);
}

//set this node's address
void RFM69_setAddress(uint8_t addr)
{
  RFM69_writeReg(REG_NODEADRS, addr);
}

//set this node's network id
void RFM69_setNetwork(uint16_t networkID)
{
  RFM69_writeReg(REG_SYNCVALUE1, (uint8_t)(networkID & 0x00FF));
  RFM69_writeReg(REG_SYNCVALUE2, (uint8_t)(networkID >> 8));
}

// set *transmit/TX* output power: 0=min, 31=max
// this results in a "weaker" transmitted signal, and directly results in a lower RSSI at the receiver
// the power configurations are explained in the SX1231H datasheet (Table 10 on p21; RegPaLevel p66): http://www.semtech.com/images/datasheet/sx1231h.pdf
// valid powerLevel parameter values are 0-31 and result in a directly proportional effect on the output/transmission power
// this function implements 2 modes as follows:
//       - for RFM69W the range is from 0-31 [-18dBm to 13dBm] (PA0 only on RFIO pin)
//       - for RFM69HW the range is from 0-31 [5dBm to 20dBm]  (PA1 & PA2 on PA_BOOST pin & high Power PA settings - see section 3.3.7 in datasheet, p22)
void RFM69_setPowerLevel(uint8_t powerLevel)
{
  _powerLevel = (powerLevel > 31 ? 31 : powerLevel);
  if (ISRFM69HW)
  {
    _powerLevel /= 2;
  }
  RFM69_writeReg(REG_PALEVEL, (RFM69_readReg(REG_PALEVEL) & 0xE0) | _powerLevel);
}




// To enable encryption: radio.encrypt("ABCDEFGHIJKLMNOP");
// To disable encryption: radio.encrypt(null) or radio.encrypt(0)
// KEY HAS TO BE 16 bytes !!!
void RFM69_encrypt(const char* key)
{
  RFM69_setMode(RF69_MODE_STANDBY);
  if (key != 0)
  {
    RFM69_select();
    SPI_transfer8(REG_AESKEY1 | 0x80);
    for (uint8_t i = 0; i < 16; i++)
      SPI_transfer8(key[i]);
    RFM69_unselect();
  }
  RFM69_writeReg(REG_PACKETCONFIG2, (RFM69_readReg(REG_PACKETCONFIG2) & 0xFE) | (key ? 1 : 0));
}


// get the received signal strength indicator (RSSI)
int16_t RFM69_readRSSI(uint8_t forceTrigger)
{
  int16_t rssi = 0;
  if (forceTrigger)
  {
    // RSSI trigger not needed if DAGC is in continuous mode
    RFM69_writeReg(REG_RSSICONFIG, RF_RSSI_START);


    while ((RFM69_readReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00); // wait for RSSI_Ready
  }
  uint8_t value = RFM69_readReg(REG_RSSICONFIG);
  rssi = RFM69_readReg(REG_RSSIVALUE);
  rssi >>= 1;
  return rssi;
}



// for RFM69HW only: you must call RFM69_setHighPower(true) after initialize() or else transmission won't work
void RFM69_setHighPower(uint8_t onOff) {
  RFM69_writeReg(REG_OCP, ISRFM69HW ? RF_OCP_OFF : RF_OCP_ON);
  if (ISRFM69HW) // turning ON
    RFM69_writeReg(REG_PALEVEL, (RFM69_readReg(REG_PALEVEL) & 0x1F) | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON); // enable P1 & P2 amplifier stages
  else
    RFM69_writeReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | _powerLevel); // enable P0 only
}

// internal function
void RFM69_setHighPowerRegs(uint8_t onOff)
{
  RFM69_writeReg(REG_TESTPA1, onOff ? 0x5D : 0x55);
  RFM69_writeReg(REG_TESTPA2, onOff ? 0x7C : 0x70);
}



uint8_t RFM69_readTemperature(uint8_t calFactor) // returns centigrade
{
  RFM69_setMode(RF69_MODE_STANDBY);
  RFM69_writeReg(REG_TEMP1, RF_TEMP1_MEAS_START);
  while ((RFM69_readReg(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING));
  return ~RFM69_readReg(REG_TEMP2) + COURSE_TEMP_COEF + calFactor; // 'complement' corrects the slope, rising temp = rising val
} // COURSE_TEMP_COEF puts reading in the ballpark, user can add additional correction


void RFM69_rcCalibration()
{
  RFM69_writeReg(REG_OSC1, RF_OSC1_RCCAL_START);
  while ((RFM69_readReg(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00);
}


void RFM69_reset(void) {
  GPIOB->MODER |= 0x400000;
  GPIOB->ODR |= 0x800;
  nano_wait(10005000);      // Wait a little over 10 ms
  GPIOB->ODR &= ~0x800;
}

uint8_t RFM69_canSend(void) {
  int16_t rssi = -RFM69_readRSSI(1);
  return (rssi < CSMA_LIMIT);
}


void RFM69_sendPacket(uint8_t RFM_targetID, uint8_t packetType, uint8_t* packet, uint8_t packetSize)
{
  // Don't send if packetSize is bigger than 61 bytes
  if (packetSize > RF69_MAX_DATA_LEN) { return; }

  RFM69_interruptOFF();

  // Wait for medium to be clear
  //while (!RFM69_canSend());

  // avoid RX deadlocks
  RFM69_writeReg(REG_PACKETCONFIG2, (RFM69_readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART);

  // turn off receiver to prevent reception while filling fifo
  RFM69_setMode(RF69_MODE_STANDBY);

  // Wait for ModeReady
  while (!RFM69_ModeReady());

  // Map DIO0 to "Packet Sent"
  RFM69_writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00);


  // Write header and packet to FIFO
  RFM69_SetCSPin(0);
  //SPI_Write(SPI2, header, rxBuffer, 4);
  SPI_transfer8(REG_FIFO | 0x80);
  SPI_transfer8(packetSize + 3);
  SPI_transfer8(RFM_targetID);
  SPI_transfer8(nodeAddress);
  SPI_transfer8(packetType);


  // Write the packet to FIFO
  //SPI_Write(SPI2, packet, rxBuffer, packetSize);
  for (uint8_t i = 0; i < packetSize; i++)
      SPI_transfer8(packet[i]);
  RFM69_SetCSPin(1);

  // Transmit the Packet
  RFM69_setMode(RF69_MODE_TX);

  // Wait for Packet Sent
  while (!RFM69_ReadDIOPin(0));

  // Set RFM69 to receive mode
  RFM69_setMode(RF69_MODE_RX);

  RFM69_interruptON();
}

uint8_t RFM69_handleReceivedPacket(uint8_t packet[74]) {
  if (RFM69_readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY) {
      // Disable
      //RFM69_interruptOFF();

      // Turn off Receiver
      RFM69_setMode(RF69_MODE_STANDBY);


      // Read FIFO
      RFM69_select();
      SPI_transfer8(REG_FIFO & 0x7F);
      uint8_t RFM_packetSize = SPI_transfer8(0);
      RFM_packetSize = RFM_packetSize > 66 ? 66 : RFM_packetSize;
      uint8_t RFM_targetID = SPI_transfer8(0);

      // Check the received target ID matches
      if(!(RFM_targetID == nodeAddress || RFM_targetID == RF69_BROADCAST_ADDR)) {
          RFM69_unselect();
          RFM69_setMode(RF69_MODE_RX);
          return 0;
      }

      // Read Data Link Packet Header
      packet[0] = SPI_transfer8(0);   // RFM69 Source ID
      packet[1] = SPI_transfer8(0);   // RFM69 Packet Type

      // Read Data Link Packet Content
      for (uint8_t i = 2; i < RFM_packetSize; i++)
      {
          packet[i] = SPI_transfer8(0);
      }

      //if (packet[0] < 62) {packet[packet[0]] = 0; }
      RFM69_unselect();

      // Set Mode to RX
      RFM69_setMode(RF69_MODE_RX);

      //RFM69_interruptON();

      return RFM_packetSize - 2;



  }

  return 0;
}


void RFM69_EXTI10_Initialize(void) {
  // Configure PB10 as Input
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  GPIOB->MODER &= ~(3<<20);
  GPIOB->PUPDR |= (GPIO_PuPd_DOWN << 20);

  // Configure SYSCFG
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  // Select PB10 as the trigger source of EXTI10
  SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR3_EXTI10;
  SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PB;


  // Enable Rising Edge Trigger for EXTI10
  EXTI->RTSR |= EXTI_RTSR_TR10;

  // Disable Falling edge trigger for EXTI10
  EXTI->FTSR &= ~EXTI_FTSR_TR10;

  // Enable EXTI10 interrupt
  EXTI->IMR |= EXTI_IMR_MR10;

  // Set EXTI10 Priority to be 0
  NVIC_SetPriority(7, 0);

  // Enable EXTI10 Interrupt for RFM69 Receive
  NVIC_EnableIRQ(7);


}




uint8_t RFM69_initialize(uint8_t nodeID, uint16_t networkID)
{
  SPI(SPI2);                                        // Setup SPI2 for RFM69
  const uint8_t CONFIG[][2] =
  {
    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
    /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_55555}, // default: 4.8 KBPS
    /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_55555},
    /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_50000}, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
    /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_50000},

    /* 0x07 */ { REG_FRFMSB, (uint8_t)  RF_FRFMSB_915 },
    /* 0x08 */ { REG_FRFMID, (uint8_t)  RF_FRFMID_915 },
    /* 0x09 */ { REG_FRFLSB, (uint8_t)  RF_FRFLSB_915 },

    // looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
    // +17dBm and +20dBm are possible on RFM69HW
    // +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
    // +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
    // +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
    //* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
    //* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

    // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
    /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
    //for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
    /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 Mapping PayloadReady & PacketSent
    /* 0x26 */ { REG_DIOMAPPING2, RF_DIOMAPPING2_DIO5_11 }, // DIO5 ModeReady indication
    /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
    /* 0x29 */ { REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
    ///* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
    /* 0x2E */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
    /* 0x2F */ { REG_SYNCVALUE1, (uint8_t)(networkID & 0x00FF) },  // NETWORK ID lower 8 bits
    /* 0x30 */ { REG_SYNCVALUE2, (uint8_t)(networkID >> 8) },      // NETWORK ID higher 8 bits
    /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_NODEBROADCAST },
    /* 0x38 */ { REG_PAYLOADLENGTH, 66 }, // in variable length mode: the max frame size, not used in TX
    /* 0x39 */ { REG_NODEADRS, nodeID },
    /* 0x3A */ { REG_BROADCASTADRS, RF69_BROADCAST_ADDR},
    /* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
    /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    //for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
    {255, 0}
  };
  uint8_t i;

  RFM69_select();

  for (i = 0; CONFIG[i][0] != 255; i++)
  {
    RFM69_writeReg(CONFIG[i][0], CONFIG[i][1]);
  }
  RFM69_unselect();
  // Encryption is persistent between resets and can trip you up during debugging.
  // Disable it during initialization so we always start from a known state.
  RFM69_encrypt(0);

  RFM69_setHighPower(ISRFM69HW); // called regardless if it's a RFM69W or RFM69HW
  RFM69_setMode(RF69_MODE_STANDBY);
  while (RFM69_ReadDIOPin(5) == 0); // wait for ModeReady

  RFM69_EXTI10_Initialize();
  nodeAddress = nodeID;
  return 1;
}




/*
int main(void) {
  SPI(SPI2);
  RFM69_reset();
  if (RFM69_initialize(RF69_915MHZ, 'A', 6) != 1) {    // Node address, network ID
        return 1;
  } else {
      uint8_t VALUE = RFM69_readReg(REG_BITRATEMSB);
      const void* buffer = "Hanyu likes boys";
      for (int i = 0; i< 100; i++){
          RFM69_send((uint8_t) 'B' , buffer, 16, 0);
      }
  }
}
*/
