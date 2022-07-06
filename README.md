# Mesh Networking Design Project
This repository was designed to be used with the STM32F091RCT ARM Cortex microcontroller.

### Basic API:

* src/main.c: Contains the all interrupt setup functions, interrupt service routines, and GPS, Battery helper functions
* src/SPI.c:  SPI initialization, write & read (RFM69HCW and MP2202 TFT Display)
* src/I2C.c:  I2C setup, write, read function needed to communicate with Sparkfun Battery Baby Sitter and SAM-M8Q GPS module
* src/bluetooth.c:  UART initialization, write, read functions to communicate with HM-19 bluetooth module
* src/lcd.c:  Helper functions to communicate with MP2202 TFT Display
* src/RFM69.c:  Helper functions to communicate with RFM69HCW Radio
* src/STPswitch.c:  Helper functions to create and facilitate a MESH Spanning Tree Protocol
* src/list.c: Doubly Linked Listed Data Structure for forwarding table
