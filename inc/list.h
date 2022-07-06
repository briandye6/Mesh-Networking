#ifndef LIST_H
#define LIST_H


#include <stdint.h>

#define LINK_ACTIVE       1
#define LINK_INACTIVE    0

typedef struct Node
{
  uint8_t key;
  uint8_t value;
  int cost;               // cost = 0 for forwarding table
  struct Node * next;
  struct Node * prev;
  // doubly linked list
} ListNode;

typedef struct
{
  ListNode * head;
  ListNode * tail;
} List;

List * createList(void);
uint8_t contains(List * list, uint8_t addr);
uint8_t getValue(List * list, uint8_t key);
uint8_t getKey(List * list, uint8_t value);
void updateValue(List * list, uint8_t key, uint8_t newValue, uint8_t newCost);
int getCost(List * list, uint8_t key);

void deleteList(List * list);
void addNode(List * list, uint8_t key, uint8_t value, int cost);
uint8_t deleteNode(List * list, uint8_t addr);
uint8_t linkActive(List * list, uint8_t addr);
void updateLinkStatus(List * list, uint8_t addr, uint8_t newStatus, uint16_t cost);


#endif
