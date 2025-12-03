// this one is meant for DB4 wavelet
// we'll use v2 as well

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "wave_objects.h"
#include <string.h> // put at top of file

void copy_reverse(const double *in, int N, double *out)
{
    for (int count = 0; count < N; count++)
    {
        out[count] = in[N - count - 1];
    }
}

void qmf_wrev(const double *in, int N, double *out)
{
    double sigOutTemp[N]; // This assumes N is known or bounded safely

    qmf_even(in, N, sigOutTemp); // Perform operations on the local array
    copy_reverse(sigOutTemp, N, out);

    return; // No need to free memory as stack-based memory is deallocated automatically
}

void qmf_even(const double *in, int N, double *out)
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

void copy(const double *in, int N, double *out)
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
int filtcoef(const int N, double *lp_d, double *hp_d, double *lp_r, double *hp_r)
{
    copy_reverse(db4, N, lp_d);
    qmf_wrev(db4, N, hp_d);
    copy(db4, N, lp_r);
    qmf_even(db4, N, hp_r);

    return N;
}

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
    int filt_len = 8; // Hardcoded, replace with a real function later!
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
    obj->outlength = siglength + 2 * J * (wave->filtlength + 1); //16 + 2 * 3 * (8 + 1) = 70 for db4 and 3 levels
    obj->J = J;
    obj->MaxIter = J; // Example
    obj->even = (siglength % 2 == 0);
    strcpy(obj->method, method);
    obj->output = &obj->dwt_coeff[0]; // Assign start of dwt_coeff as the output buffer

    // Zero the output buffer efficiently
    memset(obj->dwt_coeff, 0, (size_t)obj->outlength * sizeof(obj->dwt_coeff[0]));

    return obj;
}

void wt_free(wt_object obj)
{
    if (obj != NULL)
    {
        int index = obj - wt_pool; // Calculate index in the pool
        if (index >= 0 && index < MAX_WT_OBJECTS)
        {
            wt_pool_used[index] = 0; // Mark slot as free
        }
    }
}
