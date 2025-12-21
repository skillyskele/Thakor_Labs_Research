#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <stdint.h>
#include "wavedec.h"
#include "compression.h"



/** this stuff is meant to be in the embedded system code, declared globally or in common.h or something/
#define NUM_LEVELS 5 // should be floor(log2(SIGNAL_LENGTH))
#define TEST_WAVELET "db4"
#define NUM_CHANNELS 1
#define NUM_SAMPLES 533130


wave_object_t wave_pool[MAX_WAVE_OBJECTS];
int wave_pool_used[MAX_WAVE_OBJECTS] = {0}; // Track usage


wt_object_t wt_pool[MAX_WT_OBJECTS];
int wt_pool_used[MAX_WT_OBJECTS] = {0}; // Track usage

*/

void compress(wave_object wave, wt_object wave_transform, COEFFICIENT_TYPE cr, COMPRESSION_TYPE* data, int signal_length, int num_levels, int num_channels,
              CodewordEntry* codeword_entries, int* num_codewords, COEFFICIENT_TYPE* final_quant, int *compressed_signal_length)
{


    COEFFICIENT_TYPE tolerance = 0.1 * cr;

    COEFFICIENT_TYPE quant;


    // grab mean of each channel of the data
    // save the means used to demean the data later
    COMPRESSION_TYPE means[num_channels];
    for (int i = 0; i < num_channels; i++) {
        COMPRESSION_TYPE sum = 0;
        for (int j = 0; j < signal_length; j++) {
            sum += data[i*signal_length + j];
        }
        COMPRESSION_TYPE mean = sum / signal_length;
        means[i] = mean;
        // demean the data
        for (int j = 0; j < signal_length; j++) {
            data[i*signal_length + j] -= mean;
        }
    }


    int output_length;
    dwt(wave_transform, &data[0]);
    output_length = wave_transform->outlength;
    COEFFICIENT_TYPE sparse_rep[num_channels * output_length];


    // Store channel 0's coefficients
    for (int j = 0; j < output_length; j++)
    {
        sparse_rep[0 * output_length + j] = wave_transform->dwt_coeff[j];
    }


    // Process and store remaining channels
    // for (int i = 1; i < num_channels; i++)
    // {
    //     dwt(wave_transform, &data[i * signal_length]);
    //     for (int j = 0; j < output_length; j++)
    //     {
    //         sparse_rep[i * output_length + j] = wave_transform->dwt_coeff[j];
    //     }
    // }   // COMMENTED OUT SINCE WE ONLY HAVE ONE CHANNEL




    // this section of code sets up the binary search
    // mn is 1e-20 initially
    COEFFICIENT_TYPE mn = 1e-20;
    COEFFICIENT_TYPE mx = 0.0;
    // find max positive value in sparse_rep
    for (int i = 0; i < num_channels; i++) {
        for (int j = 0; j < output_length; j++) {
            if (sparse_rep[i * output_length + j] > mx) {
                mx = sparse_rep[i * output_length + j];
            }
        }
    }


    quant = mx;


    //COEFFICIENT_TYPE original_energy = compute_energy(sparse_rep, num_channels, output_length);


    bool searching = true;

    // flatten the temp_sparse_rep
    COEFFICIENT_TYPE temp_sparse_rep[num_channels * output_length]; // flattened version

    COEFFICIENT_TYPE q_temp;
    int32_t q_max;
    int num_nnz;
    int num_nnz_bits;


    COEFFICIENT_TYPE bpp; // bits per pixel
    int iterations = 0;


    // make a thing of 'codewords' that represent the non zero coefficients
    int32_t codewords[num_channels * output_length]; // should be large enough
    while (searching)
    {
        // quantize sparse_rep with current quant value
        q_max = 0;
        for (int i = 0; i < num_channels; i++) {
            for (int j = 0; j < output_length; j++) {
                q_temp = (sparse_rep[i * output_length + j]/quant); // sparse_rep./quant
                codewords[i * output_length + j] = (int32_t) q_temp; // fix(sparse_rep./quant), or temp_sparse_rep / quant

                temp_sparse_rep[i * output_length + j] = codewords[i * output_length + j] * quant; // multiply int32_t by COEFFICIENT_TYPE (float) to get float
                // q_max can be found here
                if (q_max < (int32_t) q_temp) {
                    q_max = round(q_temp); // include math.h
                }
            }
        }


        num_nnz_bits = 0; // reset these
        num_nnz = 0;
        for (int i = 0; i < num_channels; i++) {
            for (int j = 0; j < output_length; j++) {
                if (temp_sparse_rep[i * output_length + j] != 0) {
                    num_nnz += 1;
                }
            }
        }
        num_nnz_bits = num_nnz * ceil(log2(q_max) + 1); // bits needed to represent each non zero coefficient
        bpp = (COEFFICIENT_TYPE) num_nnz_bits / (num_channels * output_length);

        if (bpp > cr) {
            mn = quant;
            quant = (quant + mx) / 2.0;
        }
        if (bpp < cr) {
            mx = quant;
            quant = (quant + mn) / 2.0;
        }


        if (((cr - tolerance) < bpp) && (bpp < (cr + tolerance)) || iterations > 50) {
            searching = false;

            // fill up the codeword entries, and the number of codewords
            *num_codewords = num_nnz;
            num_nnz = 0; // reset this
            for (int i = 0; i < num_channels; i++) {
                for (int j = 0; j < output_length; j++) {
                    if (temp_sparse_rep[i * output_length + j] != 0) {
                        codeword_entries[num_nnz].idx = i * output_length + j;
                        codeword_entries[num_nnz].codeword = codewords[i * output_length + j];
                        num_nnz += 1;
                    }
                }
            }
            *final_quant = quant;

            *compressed_signal_length = output_length;
        }


        iterations += 1;

    }

}


