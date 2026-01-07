// this one is meant for DB4 wavelet
// we'll use v2 as well


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "wavedec.h"
#include "compression.h"
#include <string.h> // put at top of file






void copy_reverse(const COEFFICIENT_TYPE *in, int N, COEFFICIENT_TYPE *out)
{
    for (int count = 0; count < N; count++)
    {
        out[count] = in[N - count - 1];
    }
}


void qmf_wrev(const COEFFICIENT_TYPE *in, int N, COEFFICIENT_TYPE *out)
{
    COEFFICIENT_TYPE sigOutTemp[N]; // This assumes N is known or bounded safely


    qmf_even(in, N, sigOutTemp); // Perform operations on the local array
    copy_reverse(sigOutTemp, N, out);


    return; // No need to free memory as stack-based memory is deallocated automatically
}


void qmf_even(const COEFFICIENT_TYPE *in, int N, COEFFICIENT_TYPE *out)
{
    for (int count = 0; count < N; count++)
    {
        out[count] = in[N - count - 1];
        if (count % 2 != 0)
        {
            out[count] = -1 * out[count];
        }
    }
}


void copy(const COEFFICIENT_TYPE *in, int N, COEFFICIENT_TYPE *out)
{
    for (int count = 0; count < N; count++)
        out[count] = in[count];
}


/**
 *  Filter coefficient initialization (DB4 wavelet-specific for now)
 *  @param N Length of the filter (should be 8 for DB4)
 *  @param lp_d Pointer to store low-pass decomposition coefficients
 *  @param hp_d Pointer to store high-pass decomposition coefficients
 *  @param lp_r Pointer to store low-pass reconstruction coefficients
 *  @param hp_r Pointer to store high-pass reconstruction coefficients
 *  @return 0 on success
 */
int filtcoef(const int N, COEFFICIENT_TYPE *lp_d, COEFFICIENT_TYPE *hp_d, COEFFICIENT_TYPE *lp_r, COEFFICIENT_TYPE *hp_r)
{
    copy_reverse(db4, N, lp_d);
    qmf_wrev(db4, N, hp_d);
    copy(db4, N, lp_r);
    qmf_even(db4, N, hp_r);


    return N;
}


// returns a pointer to a wave_object_t struct, which is a slot in the wave_pool
wave_object wave_init(const char *wname)
{
    wave_object obj = NULL;



    // Find an unused slot in the pool
    // int i = 0;
    for (int i = 0; i < MAX_WAVE_OBJECTS; i++)
    {
        if (!wave_pool_used[i])
        {
            wave_pool_used[i] = 1; // Mark slot as used
            obj = &wave_pool[i];   // Get a reference to the slot
            break;
        }
    }
    if (obj == NULL)
    {
        return NULL; // No available WT object slots
    }


    // Initialize the slot
    int filt_len = DB4_FILTER_LENGTH;
    obj->lpd_len = obj->hpd_len = obj->lpr_len = obj->hpr_len = obj->filtlength = filt_len;


    strcpy(obj->wname, wname); // Copy wavelet name
    if (wname != NULL)
    {
        filtcoef(filt_len, obj->filter_coeff, obj->filter_coeff + filt_len, obj->filter_coeff + 2 * filt_len, obj->filter_coeff + 3 * filt_len);
    }


    // Assign pointers to specific parts of the preallocated filter_coeff memory
    obj->lpd = &obj->filter_coeff[0];
    obj->hpd = &obj->filter_coeff[filt_len];
    obj->lpr = &obj->filter_coeff[2 * filt_len];
    obj->hpr = &obj->filter_coeff[3 * filt_len];


    return obj; // Return the newly allocated object from the pool
}




void wave_free(wave_object obj)
{
    if (obj != NULL)
    {
        int index = obj - wave_pool; // Calculate index in the pool
        if (index >= 0 && index < MAX_WAVE_OBJECTS)
        {
            wave_pool_used[index] = 0; // Mark slot as free
        }
    }
}


wt_object wt_init(wave_object wave, const char *method, int siglength, int J) // can we call it depth instead of J please...


{
    wt_object obj = NULL;


    for (int i = 0; i < MAX_WT_OBJECTS; i++)
    {
        if (!wt_pool_used[i])
        {
            obj = &wt_pool[i];
            wt_pool_used[i] = 1; // Mark slot as used
            break;
        }
    }


    if (obj == NULL)
    {
        return NULL; // No available wave object slots
    }


    // Initialize the object
    obj->wave = wave;
    obj->siglength = siglength;
    obj->J = J;
    obj->MaxIter = J; // Example
    obj->even = (siglength % 2 == 0);
    strcpy(obj->method, method);


    // malloc the length array to J+1
    obj->length = (int*)malloc(sizeof(int) * (J + 2)); // J levels + final approximation + length[0] for convenience
    if (obj->length == NULL) {
        return; // catch this error!
    }


    // calculate how long the output will be
    int i = J;
    int N = siglength;
    int lp = wave->lpd_len;
    obj->length[J + 1] = N;
    obj->outlength = 0;
    while (i > 0)
    {
        N = N + lp - 2;                // example padding: [1 2 3 4] and [1 1] calls for padding like, [1 1 2 3 4 4] just lp - 1 each side
        N = (int)ceil((float)N / 2.0); // it'll downsample by 2 every level
        // printf("DWT Level %d: Calculated length after padding and downsampling: %d\n", i, N);
        obj->length[i] = N; //16 11 9 8
        obj->outlength += obj->length[i]; // keep track of the end
        i--;
    }
    obj->length[0] = obj->length[1];
    /*
    length[0] = copy length 1, so we can hold the final approximation coefficients
    length[1] = decimated a lot
    length[2] =
    ...
    length[J] = decimated by 2 (after padding)
    length[J + 1] = input signal length
    */
    obj->outlength += obj->length[0];


    // malloc the dwt_coeff array to outlength
    obj->dwt_coeff = (COEFFICIENT_TYPE*)malloc(sizeof(COEFFICIENT_TYPE) * (obj->outlength));
    if (obj->dwt_coeff == NULL) {
        return; // catch this error!
    }
    obj->output = obj->dwt_coeff; // point output to dwt_coeff for convenience
    return obj;
}


void wt_free(wt_object obj)
{
    // free the malloc'd length and dwt_coeff arrays first
    free(obj->length);
    free(obj->dwt_coeff);
    if (obj != NULL)
    {
        int index = obj - wt_pool;
        if (index >= 0 && index < MAX_WT_OBJECTS)
        {
            wt_pool_used[index] = 0; // Mark slot as free should probably be &(wt_pool_used + index)
        }
    }
}


