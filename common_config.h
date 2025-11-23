// How many samples to capture
#ifndef COMMON_CONFIG_H_
#define COMMON_CONFIG_H_

#include <stdint.h>


#define SAMPLES_PER_BUFFER               60
#define NUM_SAMPLES               SAMPLES_PER_BUFFER // each DMA transfer is one buffer's worth
#define PACKET_SIZE               SAMPLES_PER_BUFFER * SAMPLE_TYPE // or should it be sizeof(sample_type)

#define SAMPLE_TYPE uint32_t

#define BLE_PACKET_QUEUE_SIZE 16

typedef struct BluetoothPacket {
    SAMPLE_TYPE samples[SAMPLES_PER_BUFFER];
    uint32_t packet_id;
} BluetoothPacket;

#endif // COMMON_CONFIG_H_
