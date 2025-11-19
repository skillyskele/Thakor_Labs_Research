#include "circular_buffer.h"
#include "common_config.h"

#define PACKET_SIZE ((SAMPLES_PER_BUFFER + 1) * sizeof(uint32_t))

// Private static instance
static CircularBuffer buffer_instance;
static uint32_t buffer_storage[BLE_PACKET_QUEUE_SIZE * SAMPLES_PER_BUFFER];

void circular_buffer_init(int capacity) {
    buffer_instance.front = 0;
    buffer_instance.back = 0;
    buffer_instance.capacity = capacity;
    buffer_instance.num_elements = 0;
    buffer_instance.packet_history = buffer_storage;
}

void circular_buffer_enqueue(uint32_t* active_buffer) {
    if (buffer_instance.num_elements == buffer_instance.capacity) {
        return;
    }

    volatile uint32_t* dest = &buffer_instance.packet_history[buffer_instance.back * (SAMPLES_PER_BUFFER + 1)]; // in the future it'll be SAMPLES_PER_BUFFER + HEADER_SIZE but header size is just 1 for now since it's only the packet ID
    memcpy(dest, active_buffer, PACKET_SIZE);

    buffer_instance.back++;
    buffer_instance.back %= buffer_instance.capacity;
    buffer_instance.num_elements++;
}

void circular_buffer_dequeue(void) {
    if (circular_buffer_empty()) {
        return;
    }
    buffer_instance.front++;
    buffer_instance.front %= buffer_instance.capacity;
    buffer_instance.num_elements--;
}

bool circular_buffer_empty(void) {
    return buffer_instance.num_elements == 0;
}
