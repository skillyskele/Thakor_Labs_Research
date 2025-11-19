// How many samples to capture
#define SAMPLES_PER_BUFFER               60
#define NUM_SAMPLES               SAMPLES_PER_BUFFER // each DMA transfer is one buffer's worth
#define PACKET_SIZE               SAMPLES_PER_BUFFER * sizeof(uint32_t)


#define BLE_PACKET_QUEUE_SIZE 16
