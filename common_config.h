// How many samples to capture
#ifndef COMMON_CONFIG_H_
#define COMMON_CONFIG_H_

#include <stdint.h>

// original ring buffer specs
#define NUM_SAMPLES               60 // each DMA transfer is one packet's worth of samples. one packet has NUM_SAMPLES samples.
#define SAMPLE_TYPE uint32_t
#define SAMPLE_Q_SIZE 16 // how large is the original ring buffer?

// compression specs
#define N_COMPRESSION   4
#define COMPRESSION_TYPE uint32_t
#define COMPRESSED_Q_SIZE 8



typedef struct BluetoothPacket {

} BluetoothPacket;

typedef struct CompressedPacket {
} CompressedPacket;

#endif // COMMON_CONFIG_H_
