/*
 * RFM69.h
 *
 *  Created on: Feb 21, 2022
 *      Author: brian
 */

#ifndef RFM69_h
#define RFM69_h


#define RF69_MAX_DATA_LEN       61 // to take advantage of the built in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 3 bytes overhead - 2 bytes crc)
#define RF69_SPI_CS             SS // SS is the SPI slave select pin, for instance D10 on ATmega328


#define RF69_IRQ_PIN          2
#define RF69_IRQ_NUM          0


#define CSMA_LIMIT              -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP         0 // XTAL OFF
#define RF69_MODE_STANDBY       1 // XTAL ON
#define RF69_MODE_SYNTH         2 // PLL ON
#define RF69_MODE_RX            3 // RX MODE
#define RF69_MODE_TX            4 // TX MODE

// available frequency bands
#define RF69_315MHZ            31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ            43
#define RF69_868MHZ            86
#define RF69_915MHZ            91

#define null                  0
#define COURSE_TEMP_COEF    -90 // puts the temperature reading in the ballpark, user can fine tune the returned value
#define RF69_BROADCAST_ADDR 255
#define RF69_CSMA_LIMIT_MS 1000
#define RF69_TX_LIMIT_MS   1000
#define RF69_FSTEP  61.03515625 // == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet)

// TWS: define CTLbyte bits
#define RFM69_CTL_SENDACK   0x80
#define RFM69_CTL_REQACK    0x40

//
#define ISRFM69HW  1

#define DATA_PACKET       0
#define CONTROL_PACKET    1


// module interface, platform specific
#include <stdint.h>
void RFM69_interruptOFF();
void RFM69_interruptON();
uint8_t RFM69_initialize(uint8_t nodeID, uint16_t networkID);
uint8_t RFM69_receiveDone();
void RFM69_send(uint8_t toAddress, uint8_t * buffer, uint8_t bufferSize, uint8_t requestACK);
void RFM69_SetCSPin(uint8_t value);
void RFM69_writeReg(uint8_t addr, uint8_t value);
uint8_t RFM69_readReg(uint8_t addr);
void RFM69_Sleep(void);

void RFM69_setAddress(uint8_t addr);
void RFM69_reset(void);
void RFM69_setHighPower(uint8_t onOff);
void RFM69_setHighPowerRegs(uint8_t onOff);
void RFM69_setMode(uint8_t newMode);
uint8_t RFM69_getMode(void);
void RFM69_encrypt(const char* key);
int16_t RFM69_readRSSI(uint8_t forceTrigger);
uint16_t RFM69_ReadDIOPin(uint8_t pin);
uint8_t RFM69_canSend(void);
void RFM69_sendPacket(uint8_t RFM_targetID,uint8_t packetType, uint8_t* packet, uint8_t packetSize);
uint8_t RFM69_handleReceivedPacket(uint8_t packet[74]);
void RFM69_EXTI10_Initialize(void);




#endif
