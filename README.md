# ENIGMA Mesh Networking Design Project
This repository was designed to be used with the STM32F091RCT ARM Cortex microcontroller.

### Repository Layout:
| File          | Purpose         |
| ------------- |:-------------| 
| src/main.c    | Contains the all interrupt setup functions, interrupt service routines, and GPS, Battery helper functions |
| src/SPI.c     | SPI initialization, write, and read for RFM69HCW Radio and MP2202 TFT Display    |
| src/I2C.c     | I2C initialization, write, and read for Sparkfun Battery Baby Sitter and SAM-M8Q GPS module|
| src/bluetooth.c|  UART initialization, write, and read for HM-19 bluetooth module|
| src/lcd.c|  Helper functions to communicate with MP2202 TFT Display|
| src/RFM69.c|  Helper functions to communicate with RFM69HCW Radio|
| src/STPswitch.c|  Helper functions to create and facilitate a MESH Spanning Tree Protocol|
| src/list.c| Doubly Linked Listed Data Structure for forwarding table|


### User Interace: Progressive Web Application
https://enigmaterminal.net/

### Final Design Review Presentation Link with VIDEO DEMONSTRATION
https://purdue0-my.sharepoint.com/:p:/g/personal/dye28_purdue_edu/Ee8eXgqAICREm-nx36Vz36ABuLT1QJ_dSffJ5avcmjQzwg?e=PEjIST
