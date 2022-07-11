/*
 * STPswitch.c
 *
 *  Created on: Dec 25, 2021
 *      Author: brian
 */

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "STPswitch.h"
#include "list.h"
#include "RFM69.h"
#include "bluetooth.h"



//extern STPswitch * switchNode;
extern uint8_t nodeAddress;


STPswitch * createSTPswitch(uint8_t addr) {
  STPswitch * switchNode = malloc(sizeof(STPswitch));           // Allocate memory for Switch
  switchNode->addr = addr;
  switchNode->_H = addr;
  switchNode->_R = addr;
  switchNode->cost_to_root = 0;
  switchNode->forwarding_table = createList();    // {(MAC A, PORT), (MAC B, PORT) ... }
  switchNode->forwarding_table->head = NULL;
  switchNode->forwarding_table->tail = NULL;
  switchNode->links = createList();               // {(PORT, STATUS), (PORT, STATUS) ... }
  switchNode->links->head = NULL;
  switchNode->links->tail = NULL;
  switchNode->connected = 0;
  return switchNode;
}


// Send Packet to every link that is active and not the port in which it
// was received.

void broadcast(STPswitch * switchNode, uint8_t recvAddr, uint8_t * packet, uint8_t packetSize) {
  ListNode * link = switchNode->links->head;
  while (link != NULL) {
      if (link->key != recvAddr && link->value == LINK_ACTIVE) {

          RFM69_sendPacket(link->key, DATA_PACKET, packet, packetSize);

      }
      link = link->next;
  }
}



void sendControlPacket(STPswitch * switchNode) {
  uint8_t cost_H = (uint8_t) (((switchNode->cost_to_root) & 0xFF00) >> 8);
  uint8_t cost_L = (uint8_t) ((switchNode->cost_to_root) & 0x00FF);

  uint8_t controlPacket[5] = {  switchNode->_R,
                                cost_H,
                                cost_L,
                                nodeAddress,
                                switchNode->_H };

  RFM69_sendPacket(RF69_BROADCAST_ADDR, CONTROL_PACKET, controlPacket, 5);

}


// Spanning Tree Protocol
void stp(STPswitch * switchNode, uint8_t * controlPacket, uint16_t cost_to_Y) {
  switchNode->connected = 1;

  // Parse contents of packet
  uint8_t adv_R = controlPacket[2];
  uint16_t adv_cost = (uint16_t) (controlPacket[3] << 8) | controlPacket[4];
  uint8_t adv_Y = controlPacket[5];     // Advertising Neighbor = RFM69sourceID
  uint8_t adv_H = controlPacket[6];

  // Calculate the cost to adv_R via adv_Y
  uint16_t potential_cost = cost_to_Y + adv_cost;

  // Case 0: Split Horizon
  if (adv_H == nodeAddress || adv_R == nodeAddress) {
      updateLinkStatus(switchNode->links, adv_Y, LINK_ACTIVE, cost_to_Y);
      //updateValue(switchNode->forwarding_table, adv_Y, adv_Y, 0);
      return;

  // Spanning Tree Protocol (STP)
  // Case #1: Advertisement is from the CURRENT next hop to root
  } else if ((switchNode->_H ==  adv_Y) || (switchNode->_R == adv_R)) {
      switchNode->_R = adv_R;
      switchNode->_H = adv_Y;
      switchNode->cost_to_root = potential_cost;
      updateLinkStatus(switchNode->links, adv_Y, LINK_ACTIVE, cost_to_Y);
      //sendControlPacket(switchNode);


  // Case #2: Advertisement is from a switch that is not the current next hop neighbor
  } else if (switchNode->_H != adv_Y) {

      // Smaller Root Advertised
      if (adv_R < switchNode->_R) {
          switchNode->_R = adv_R;
          switchNode->_H = adv_Y;
          switchNode->cost_to_root = potential_cost;
          updateLinkStatus(switchNode->links, adv_Y, LINK_ACTIVE, cost_to_Y);
          sendControlPacket(switchNode);


      // Same Root but lower cost
      } else if (adv_R == switchNode->_R && potential_cost < switchNode->cost_to_root) {
          switchNode->_R = adv_R;
          switchNode->_H = adv_Y;
          switchNode->cost_to_root = potential_cost;
          updateLinkStatus(switchNode->links, adv_Y, LINK_ACTIVE, cost_to_Y);
          sendControlPacket(switchNode);

      // Same Root Same Cost but advertiser has smaller address
      } else if (adv_R == switchNode->_R && potential_cost == switchNode->cost_to_root && adv_Y < switchNode->_H) {
          switchNode->_R = adv_R;
          switchNode->_H = adv_Y;
          switchNode->cost_to_root = potential_cost;
          updateLinkStatus(switchNode->links, adv_Y, LINK_ACTIVE, cost_to_Y);
          sendControlPacket(switchNode);

      } else if (adv_H != nodeAddress) {
          updateLinkStatus(switchNode->links, adv_Y, LINK_INACTIVE, cost_to_Y);
          // Don't want to delete the node from forwarding table because we still
          // want to send control packets

      } else {
          updateLinkStatus(switchNode->links, adv_Y, LINK_ACTIVE, cost_to_Y);
      }
  }

}

// packet format/structure:
// packet[0] = RFM Source ID
// packet[1] = packetType = DATA/CONTROL
// DATA     packet[2:66] = {src, dst, port, checksum, data0, data1, ... }
// CONTROL  packet[2:5]  = {ROOT, cost_to_root, adv_Y, adv_H}

void handlePacket(STPswitch * switchNode, uint8_t packet[74], uint8_t packetSize) {

  if (packetSize == 0) { return; }

  List * forwardingTable = switchNode->forwarding_table;
  List * links = switchNode->links;
  uint8_t recvAddr = packet[0];

  // Read the RSSI of the received packet
  int16_t rssi = RFM69_readRSSI(0);



  // Packet is a CONTROL packet
  if (packet[1] == CONTROL_PACKET) {
      stp(switchNode, packet, rssi);


  // Packet is a DATA packet
  } else {

      // MAC Learning
      uint8_t srcAddr = packet[2];
      // Update nextHop in forwarding table
      if (recvAddr != 0) {
          updateValue(forwardingTable, srcAddr, recvAddr, 0);
      }

      uint8_t port = packet[4];
      if (port == 51) {
          packet[packetSize] = nodeAddress;
          packet[5] = packet[5] + 1;
      }

      uint8_t dstAddr = packet[3];
      // Packet received was for US
      if (dstAddr == nodeAddress) {
          USART4_Write(&(packet[2]), packet[5]);    // Write UDP Packet to Host
          USART4_Write((uint8_t *) "\n", 1);        // Write Terminating newline character


      // Broadcast packet was received
      } else if (dstAddr == RF69_BROADCAST_ADDR) {
          broadcast(switchNode, recvAddr, &(packet[2]), packet[5]);

      // Check if destination address is in forwarding table
      } else if (contains(forwardingTable, dstAddr)) {
          uint8_t nextHop = getKey(forwardingTable, dstAddr);

          // Check if the nextHop to dstAddr is in our links
          if (!contains(links, dstAddr)) {
              // Drop Packet
              return;
          }

          // If the next hop to dstAddr is the node in which it came from, drop packet
          // Or if the nextHop Link is not active, drop packet
          if (nextHop == recvAddr || !linkActive(links, dstAddr)) {
              return;

          // Else, send packet to next hop
          } else {
              RFM69_sendPacket(nextHop, DATA_PACKET, &(packet[2]), packet[5]);
          }

      // Else if no entry in forwarding table corresponds to dstAddr exists, broadcast
      } else {
          broadcast(switchNode, recvAddr, &(packet[2]), packet[5]);
      }

  }

}


// New Spanning Tree Link with endpoint
void handleNewLink(STPswitch * switchNode, uint8_t recvAddr, uint8_t endpoint, uint16_t cost) {

  if (endpoint == switchNode->_H) {
      uint16_t prev_cost = getCost(switchNode->links, endpoint);
      switchNode->cost_to_root = switchNode->cost_to_root - prev_cost + cost;
  }

  updateLinkStatus(switchNode->links, endpoint, LINK_ACTIVE, cost);
  sendControlPacket(switchNode);

}

/*
void handleRemoveLink(STPswitch * switchNode, uint32_t freq, char * endpoint) {

  if (strcmp(switchNode->_H, endpoint) == 0) {
      switchNode->_H = switchNode->addr;
      switchNode->_R = switchNode->addr;
      switchNode->cost_to_root = 0;

  }

  deleteNode(switchNode->forwarding_table, endpoint);
  sendControlPacket(switchNode);

}
*/

void handlePeriodicOps(STPswitch * switchNode) {
  sendControlPacket(switchNode);
}
