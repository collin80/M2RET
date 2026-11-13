/*
 * rx_queue.cpp
 *
 * See rx_queue.h. Single producer (CAN ISR) / single consumer (main loop) ring
 * buffer per bus, so no locking is needed beyond volatile head/tail.
 */

#include "rx_queue.h"

RX_QUEUE rxQueue[2];

static void rxQueuePush(RX_QUEUE &q, CAN_FRAME *frame)
{
    uint32_t now = micros();
    uint16_t head = q.head;
    uint16_t next = (head + 1) % RX_QUEUE_SIZE;
    q.received++;
    if (next == q.tail) { //queue full, drop the newest frame and count it
        q.dropped++;
        return;
    }
    q.entries[head].frame = *frame;
    q.entries[head].timestamp = now;
    q.head = next;
    uint16_t count = (next + RX_QUEUE_SIZE - q.tail) % RX_QUEUE_SIZE;
    if (count > q.highWater) q.highWater = count;
}

void rxCan0ISR(CAN_FRAME *frame) { rxQueuePush(rxQueue[0], frame); }
void rxCan1ISR(CAN_FRAME *frame) { rxQueuePush(rxQueue[1], frame); }

uint16_t rxQueueCount(int bus)
{
    uint16_t h = rxQueue[bus].head;
    uint16_t t = rxQueue[bus].tail;
    return (h + RX_QUEUE_SIZE - t) % RX_QUEUE_SIZE;
}

int rxQueuePopOldest(CAN_FRAME &frame, uint32_t &timestamp)
{
    int bus;
    bool has0 = rxQueueCount(0) > 0;
    bool has1 = rxQueueCount(1) > 0;

    if (has0 && has1) {
        //signed difference copes with micros() wrapping
        int32_t diff = (int32_t)(rxQueue[0].entries[rxQueue[0].tail].timestamp - rxQueue[1].entries[rxQueue[1].tail].timestamp);
        bus = (diff <= 0) ? 0 : 1;
    }
    else if (has0) bus = 0;
    else if (has1) bus = 1;
    else return -1;

    RX_QUEUE &q = rxQueue[bus];
    uint16_t t = q.tail;
    frame = q.entries[t].frame;
    timestamp = q.entries[t].timestamp;
    q.tail = (t + 1) % RX_QUEUE_SIZE;
    return bus;
}

void rxQueueResetStats()
{
    for (int b = 0; b < 2; b++) {
        rxQueue[b].received = 0;
        rxQueue[b].dropped = 0;
        rxQueue[b].highWater = rxQueueCount(b);
    }
}
