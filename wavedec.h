#ifndef WAVEDEC_H // Include guard to prevent multiple inclusions
#define WAVEDEC_H


#include "compression_types.h"


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>


#ifdef __cplusplus
extern "C"
{
#endif


// Constants
#define MAX_WAVE_OBJECTS 1 // Maximum number of wave objects in the pool
#define MAX_FILTER_SIZE 256 // Maximum length of the filter stored in wave_object
#define MAX_WT_OBJECTS 1   // Maximum number of DWT objects
#define DB4_FILTER_LENGTH 8 // Filter length for DB4 wavelet
#define MAX_LEVELS 10
#define MAX_SIG_LENGTH 512
#define MAX_COEFF_SIZE (MAX_SIG_LENGTH + 2 * MAX_LEVELS * (DB4_FILTER_LENGTH + 1)) // Max size of dwt_coeff


    // Static global objects pool declarations
    typedef struct wave_set
    {
        char wname[20];                           // Name of the wavelet
        int filtlength;                           // Length of the filter
        int lpd_len;                              // Length of low-pass decomposition filter
        int hpd_len;                              // Length of high-pass decomposition filter
        int lpr_len;                              // Length of low-pass reconstruction filter
        int hpr_len;                              // Length of high-pass reconstruction filter
        COEFFICIENT_TYPE *lpd;                              // Low-pass decomposition filter
        COEFFICIENT_TYPE *hpd;                              // High-pass decomposition filter
        COEFFICIENT_TYPE *lpr;                              // Low-pass reconstruction filter
        COEFFICIENT_TYPE *hpr;                              // High-pass reconstruction filter
        COEFFICIENT_TYPE filter_coeff[4 * MAX_FILTER_SIZE]; // Memory for 4 sets of filter coefficients
    } wave_object_t;


    typedef wave_object_t *wave_object; // Pointer to `wave_set`

    typedef struct wt_set
    {
        wave_object wave;                  // Pointer to associated wavelet object
        char method[10];                   // Method used (e.g., "dwt")
        int *length;                       // it should be as long as J + 1
        int lenlength;                     // Usually like 2 * J + 1 cuz it goes A1 D1, A2 D2,..AJ DJ, final Approximation coefficients
        int siglength;                     // Length of the original signal
        int outlength;                     // Length of the output coefficients
        int J;                             // Number of decomposition levels
        int MaxIter;                       // Maximum iterations for decomposition
        int even;                          // Whether signal length is even (1 for even, 0 for odd)
        char ext[10];                      // Type of boundary extension ("per", "sym")
        char cmethod[10];                  // Convolution method ("direct", "FFT")
        COEFFICIENT_TYPE *output;                    // Pointer to output coefficients
        COEFFICIENT_TYPE *dwt_coeff; // Memory for wavelet coefficients
    } wt_object_t;


    typedef wt_object_t *wt_object; // Pointer to `wt_set`


    extern wave_object_t wave_pool[MAX_WAVE_OBJECTS];
    extern int wave_pool_used[MAX_WAVE_OBJECTS]; // Track usage
    extern wt_object_t wt_pool[MAX_WT_OBJECTS];
    extern int wt_pool_used[MAX_WT_OBJECTS]; // Track usage


    // Filter coefficient initialization (DB4 wavelet-specific)
    int filtcoef(const int N, COEFFICIENT_TYPE *lp_d, COEFFICIENT_TYPE *hp_d, COEFFICIENT_TYPE *lp_r, COEFFICIENT_TYPE *hp_r);


    // Wavelet representation object initialization
    wave_object wave_init(const char *wname);
    void wave_free(wave_object obj);


    // Discrete wavelet transform object initialization
    wt_object wt_init(wave_object wave, const char *method, int siglength, int J);
    void wt_free(wt_object obj);




    // Helper functions (these are static helper operations used internally)
    void copy(const COEFFICIENT_TYPE *in, int N, COEFFICIENT_TYPE *out);
    void copy_reverse(const COEFFICIENT_TYPE *in, int N, COEFFICIENT_TYPE *out);
    void qmf_even(const COEFFICIENT_TYPE *in, int N, COEFFICIENT_TYPE *out);
    void qmf_wrev(const COEFFICIENT_TYPE *in, int N, COEFFICIENT_TYPE *out);


    // these are Lo_R filter coefficients for db4
    static const COEFFICIENT_TYPE db4[DB4_FILTER_LENGTH] = {
        0.230377813308855,
        0.714846570552542,
        0.630880767929590,
        -0.0279837694169839,
        -0.187034811718881,
        0.0308413818359870,
        0.0328830116669829,
        -0.0105974017849973,};






#ifdef __cplusplus
}
#endif
#endif // WAVEDEC_H
