/*
 * rx_queue.h
 *
 * CAN receive queues filled from the due_can interrupt.
 *
 * Frames are copied out of the CAN controller inside the CAN interrupt and stamped
 * with micros() at that moment. The main loop then drains the queues in bursts.
 * This replaces reading one frame per bus per loop pass and stamping it when it was
 * sent to USB, which dropped frames on busy buses and gave timestamps that reflected
 * loop timing rather than bus timing.
 */

#ifndef RX_QUEUE_H_
#define RX_QUEUE_H_

#include <Arduino.h>
#include <due_can.h>
#include "config.h"

typedef struct {
    CAN_FRAME frame;
    uint32_t timestamp; //micros() when the frame came out of the controller
} RX_ENTRY;

typedef struct {
    RX_ENTRY entries[RX_QUEUE_SIZE];
    volatile uint16_t head; //written only by the ISR
    volatile uint16_t tail; //written only by the main loop
    volatile uint32_t received;
    volatile uint32_t dropped;
    volatile uint16_t highWater;
} RX_QUEUE;

extern RX_QUEUE rxQueue[2];

void rxCan0ISR(CAN_FRAME *frame);
void rxCan1ISR(CAN_FRAME *frame);
uint16_t rxQueueCount(int bus);
//Copies the oldest frame across both queues. Returns the bus (0/1) or -1 if both are empty.
int rxQueuePopOldest(CAN_FRAME &frame, uint32_t &timestamp);
void rxQueueResetStats();

#endif
