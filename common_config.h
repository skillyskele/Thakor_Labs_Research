// How many samples to capture
#ifndef COMMON_CONFIG_H_
#define COMMON_CONFIG_H_

#include <stdint.h>
#include "compression_types.h"


// original ring buffer specs
#define NUM_SAMPLES               10 // each DMA transfer is one packet's worth of samples. one packet has NUM_SAMPLES samples.
//typedef uint16_t SAMPLE_TYPE;
#define SAMPLE_Q_SIZE  16// how large is the original ring buffer?
typedef struct {
    SAMPLE_TYPE samples[NUM_SAMPLES];
    // Add more fields if needed (e.g., timestamp, id)
} SampleSlotType;

// compression specs
#define COMPRESSION_THRESHOLD 16
#define N_COMPRESSION   16
typedef uint16_t COMPRESSION_TYPE;
#define COMPRESSED_BUFFER_SIZE 160 // must be N_COMPRESSION * NUM_SAMPLES


#define NUM_LEVELS 4 // should be floor(log2(N_COMPRESSION))
#define TEST_WAVELET "db4"
#define NUM_CHANNELS 1
#define MAX_CODEWORDS COMPRESSION_THRESHOLD/2
#define COMPRESSION_RATIO 0.1



#endif // COMMON_CONFIG_H_
