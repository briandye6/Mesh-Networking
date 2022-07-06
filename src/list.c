#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "list.h"


// LINKED LIST DATA STRUCTURE

// Alloc memory for a new Linked List and return the pointer
List * createList(void) {
  List * LinkedList = malloc(sizeof(List));
  return LinkedList;
}


// is the key contained in the List?
uint8_t contains(List * list, uint8_t key) {
  ListNode * p = list->head;
  while (p != NULL) {
      if (p->key == key) {
          return 1;
      }
      p = p->next;
  }
  return 0;
}


// Get value at key
uint8_t getValue(List * list, uint8_t key) {
    ListNode * p = list->head;
    while (p != NULL) {
        if (p->key == key) {
            return p->value;
        }
        p = p->next;
    }
    return -1;  // Error
}


// Get the key at value
uint8_t getKey(List * list, uint8_t value) {
  ListNode * p = list->head;
  while (p != NULL) {
      if (p->value == value) {
          return p->key;
      }
      p = p->next;
  }
  return -1;  // Error
}



// Update Value at Key
void updateValue(List * list, uint8_t key, uint8_t newValue, uint8_t newCost) {
  ListNode * p = list->head;
  while (p != NULL) {
      if (p->key == key) {
          p->value = newValue;
          p->cost = newCost;
          return;
      }
      p = p->next;
  }
  addNode(list, key, newValue, 0);
}

// Get cost of link at key=addr
int getCost(List * list, uint8_t key) {
  ListNode * p = list->head;
  while (p != NULL) {
      if (p->key == key) {
          return p->cost;
      }
      p = p->next;
  }
  return 0xFFFF; // nextHop is not in links
}



// If arithlist is NULL, do nothing
// release the memory of every node in the list
// release the memory of the list 
void deleteList(List * list)
{
  if (list == NULL) { return; }
  ListNode * p = list->head;
  while (list->head != NULL) {
      list->head = list->head->next;
      //free(p->value);
      free(p);
      p = list->head;
  }
  free(list);
}



// Input: 
// arithlist stores the addresses of head and tail
// If arithlist is NULL, do nothing
// word is the word to be added
//
// Output:
// a ListNode is added to the end (become tail)
//
// allocate memory for a new ListNode
// copy word to the word attribute of the new ListNode
// insert the ListNode to the list
void addNode(List * list, uint8_t key, uint8_t value, int cost)
{
  // These condition always occur
  ListNode * newNode = malloc(sizeof(ListNode));
  newNode->key = key;
  newNode->value = value;
  newNode->cost = cost;
  newNode->next = NULL;
  newNode->prev = list->tail;


  // If the list is empty (no nodes)
  if (list->head == NULL && list->tail == NULL) {
      list->head = newNode;
      list->tail = newNode;
      list->head->prev = NULL;
      list->head->next = NULL;
      list->tail->prev = NULL;
      list->tail->next = NULL;
    return;
  }

  // tail points to new node
  // new Node prev points to tail
  // tail become new node
  list->tail->next = newNode;
  newNode->prev = list->tail;
  list->tail = newNode;
  
  
}


uint8_t linkActive(List * list, uint8_t dstAddr) {
    ListNode * point = list->head;
    while (point != NULL) {
        if (point->key == dstAddr) {
            return point->value;
        }
        point = point->next;
    }
    return false;
}

// Update the status and cost of link
void updateLinkStatus(List * list, uint8_t addr, uint8_t newStatus, uint16_t cost) {
    ListNode * link = list->head;
    while (link != NULL) {
        if (link->key == addr) {
            link->value = newStatus;
            link->cost = cost;
            return;
        }
        link = link->next;
    }
    addNode(list, addr, newStatus, cost);
}





uint8_t deleteNode(List * list, uint8_t addr)
{
  // If arithlist or ln is NULL, return false
  if (list == NULL) { return false; }

  // If the list is empty return false
  if (list->head == NULL && list->tail == NULL) { return false; }

  // Last node in list in list is the Node to delete?
  if (list->head->key == addr && list->tail->key == addr ) {
      list->head = NULL;
      list->tail = NULL;
      free(list->tail);
      return true;
  }
  
  
  // Check if Node to delete is tail
  ListNode * p1 = list->tail;
  if (p1->key == addr) {
      list->tail = list->tail->prev;
      free(p1);
      list->tail->next = NULL;
      return true;
  }

  
  // Check if node to delete is head
  p1 = list->head;
  if (p1->key == addr) {
      list->head = list->head->next;
      free(p1);
      list->head->prev = NULL;
      return true;
  }

  p1 = p1->next;
  // Scan List for Node
  while (p1 != NULL) {
    if (p1->key == addr) {
      ListNode * p2 = p1->next;
      p1->next = p2->next;
      p2->next->prev = p1;
      free(p2);
      return true;
    }
    p1 = p1->next;
  }

  return false;
}


