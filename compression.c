#include "wave_library.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "wave_library.h"
#include "common_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/**
 * Compresses BluetoothPacket into CompressedPacket using wavelet transform and quantization.
 *
 * @param src Pointer to the source BluetoothPacket structure (contains raw signal data).
 * @param dst Pointer to the destination CompressedPacket structure (stores compressed 8-bit data).
 * @param wt The wavelet transform object, globally declared in main.c.
 */
void wavedec_compress(BluetoothPacket *src, CompressedPacket *dst, wt_object wt)
{
    int siglength = wt->siglength;  // Size of data chunks to process
    int depth = wt->J;             // Decomposition levels for the DWT
    SAMPLE_TYPE *samples = src->samples; // Pointer to input signal data
    uint32_t packet_id = src->packet_id; // Source packet ID

    double *buffer = (double *)malloc(sizeof(double) * siglength); // Temporary buffer for signal processing
    if (buffer == NULL) {
        printf("Error: Unable to allocate memory for buffer\n");
        return;
    }

    // Prepare destination compressed data
    uint8_t *compressed_data = dst->compressed_data; // Pointer to store compressed 8-bit data
    int compressed_index = 0;  // Track index for filling `compressed_data`

    // Copy the packet ID into the destination compressed packet
    dst->packet_id = (uint8_t)packet_id;

    // Step 1: Convert input samples to double for processing
    for (int i = 0; i < siglength; i++) {
        buffer[i] = (double)samples[i];
    }

    // Step 2: Apply wavelet transform (DWT) on the buffer
    dwt(wt, buffer);

    // Step 3: Quantize coefficients into 8-bit values and store them into `compressed_data`
    for (int i = 0; i < wt->outlength; i++) {
        // Simple quantization example: Scale and clamp coefficients to fit into 8-bit range
        double coeff = wt->dwt_coeff[i];
        int quantized_value = (int)round((coeff + 128.0) / 2.0); // Example scaling to fit into [0, 255]
        compressed_data[compressed_index++] = (uint8_t)(quantized_value > 255 ? 255 : (quantized_value < 0 ? 0 : quantized_value));
    }

    // Step 4: Store the size of compressed data in the `dst` structure
    dst->compressed_size = compressed_index;

    // Cleanup allocated memory
    free(buffer);

    // Debug message
    printf("Compression complete: Packet ID=%d, Compressed size=%d bytes\n", packet_id, compressed_index);

    return;
}
