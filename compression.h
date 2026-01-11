#ifndef COMPRESSION_H
#define COMPRESSION_H

#include "compression_types.h"
#include "wavedec.h"

#ifdef __cplusplus
extern "C" {
#endif


void compress(wt_object wave_transform, COEFFICIENT_TYPE cr, COMPRESSION_TYPE* data, int signal_length, int num_channels,
              volatile CodewordEntry* codeword_entries, int* num_codewords, COEFFICIENT_TYPE* final_quant, int *compressed_signal_length);

#ifdef __cplusplus
}
#endif

#endif /* COMPRESSION_H */
