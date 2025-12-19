// How many samples to capture
#ifndef COMMON_CONFIG_H_
#define COMMON_CONFIG_H_

#include <stdint.h>

// original ring buffer specs
#define NUM_SAMPLES               10 // each DMA transfer is one packet's worth of samples. one packet has NUM_SAMPLES samples.
typedef uint16_t SAMPLE_TYPE;
#define SAMPLE_Q_SIZE  16// how large is the original ring buffer?
typedef struct {
    SAMPLE_TYPE samples[NUM_SAMPLES];
    // Add more fields if needed (e.g., timestamp, id)
} SampleSlotType;

// compression specs
#define COMPRESSION_THRESHOLD 4
#define N_COMPRESSION   4
typedef uint16_t COMPRESSION_TYPE;
#define COMPRESSED_BUFFER_SIZE 40 // must be N_COMPRESSION

#endif // COMMON_CONFIG_H_
