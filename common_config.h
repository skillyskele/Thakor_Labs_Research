// How many samples to capture
#ifndef COMMON_CONFIG_H_
#define COMMON_CONFIG_H_

#include <stdint.h>

// original ring buffer specs
#define NUM_SAMPLES               1 // each DMA transfer is one packet's worth of samples. one packet has NUM_SAMPLES samples.
typedef uint16_t SAMPLE_TYPE;
#define SAMPLE_Q_SIZE  128 // MUST be a power of 2!!!!!
typedef struct {
    SAMPLE_TYPE samples[NUM_SAMPLES];
    // Add more fields if needed (e.g., timestamp, id)
} SampleSlotType;

// compression specs
#define COMPRESSION_THRESHOLD 64
#define N_COMPRESSION   64
typedef uint16_t COMPRESSION_TYPE;
#define COMPRESSED_BUFFER_SIZE 64 // must be N_COMPRESSION

#endif // COMMON_CONFIG_H_
