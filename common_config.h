// How many samples to capture
#ifndef COMMON_CONFIG_H_
#define COMMON_CONFIG_H_

#include <stdint.h>
#include "compression_types.h"


// original ring buffer specs
#define NUM_SAMPLES               5 // each DMA transfer is one packet's worth of samples. one packet has NUM_SAMPLES samples.
typedef uint16_t SAMPLE_TYPE;
#define SAMPLE_Q_SIZE  32

typedef struct {
    SAMPLE_TYPE samples[NUM_SAMPLES];
    // Add more fields if needed (e.g., timestamp, id)
} SampleSlotType;

// compression specs
#define COMPRESSION_THRESHOLD 16
#define N_COMPRESSION   16
#define COMPRESS_AT_A_TIME 80 // must be N_COMPRESSION * NUM_SAMPLES


#define NUM_LEVELS 6 // should be floor(log2(COMPRESSED_BUFFER_SIZE)), always!!!
#define TEST_WAVELET "db4"
#define NUM_CHANNELS 1
#define MAX_CODEWORDS COMPRESS_AT_A_TIME/2
#define COMPRESSION_RATIO 0.1

// some bluetooth definitions, just for clarity
typedef enum {
    BLE_TRANSFER_IDLE,
    BLE_TRANSFER_SENDING_HEADER,
    BLE_TRANSFER_SENDING_DATA
} ble_transfer_state_t;


#endif // COMMON_CONFIG_H_
