#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    int front;
    int back;
    int capacity;
    int num_elements;
    uint32_t* packet_history;
} CircularBuffer;

void circular_buffer_init(int capacity);
void circular_buffer_enqueue(uint32_t* active_buffer);
void circular_buffer_dequeue(void);
bool circular_buffer_empty(void);

#endif // CIRCULAR_BUFFER_H
