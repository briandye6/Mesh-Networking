/*
 * STPswitch.h
 *
 *  Created on: Dec 25, 2021
 *      Author: brian
 */

#include "list.h"


typedef struct
{

  uint8_t addr;
  uint8_t _R;
  uint8_t _H;
  uint16_t cost_to_root;

  List * forwarding_table;    // MAC Learning & Forwarding
  List * links;               // STP active/inactive links

  uint8_t connected;
} STPswitch;


STPswitch * createSTPswitch(uint8_t addr);
void handlePacket(STPswitch * switchNode, uint8_t packet[74], uint8_t packetSize);
void handleNewLink(STPswitch * switchNode, uint8_t recvAddr, uint8_t endpoint, uint16_t cost);
void handleRemoveLink(STPswitch * switchNode, uint8_t recvAddr, char * endpoint);
void handlePeriodicOps(STPswitch * Node);
void stp(STPswitch * switchNode, uint8_t * controlPacket, uint16_t cost_to_Y);
void broadcast(STPswitch * switchNode, uint8_t recvAddr, uint8_t * Packet, uint8_t packetSize);
void sendControlPacket(STPswitch * Node);


