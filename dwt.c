#include "wavedec.h"
#include "dwt.h"
#include <math.h>


/**
 * inp is the input signal. At first it's the input signal, then it's the approximation coefficients from last level
 * N is the length of inp
 * lpd is low pass decomposition filter
 * hpd is high pass decomposition filter
 * lpd_len is length of low pass decomposition filter
 * cA is output approximation coefficients
 * len_cA is length of cA
 * cD is output detail coefficients
 * istride is input stride
 * ostride is output stride
 */
void dwt_sym_stride(COEFFICIENT_TYPE *inp, int N, COEFFICIENT_TYPE *lpd, COEFFICIENT_TYPE *hpd, int lpd_len, COEFFICIENT_TYPE *cA, int len_cA, COEFFICIENT_TYPE *cD, int istride, int ostride)
{
    int i, l, t, len_avg;
    int is, os;
    len_avg = lpd_len;


    for (i = 0; i < len_cA; ++i)
    {
        t = 2 * i + 1;
        os = i * ostride;
        cA[os] = 0.0;
        cD[os] = 0.0;
        //printf("Debug: N=%d, len_cA=%d, len_avg=%d\n", N, len_cA, len_avg); // Debug: Show N, len_cA, and t values
        for (l = 0; l < len_avg; ++l)
        {
            if ((t - l) >= 0 && (t - l) < N)
            {
                is = (t - l) * istride;
                //printf("Normal: t=%d, l=%d, is=%d, os=%d\n", t, l, is, os); // Debug: Normal access
                cA[os] += lpd[l] * inp[is];
                cD[os] += hpd[l] * inp[is];
            }
            else if ((t - l) < 0)
            {
                is = (-t + l - 1) * istride; //represents how you're growing into it. so t = 1. then take off 1 before you start reflecting, then let l grow naturally: 0, 1, 2, 3...
                //printf("Left padding: t=%d, l=%d, is=%d, os=%d\n", t, l, is, os); // Debug: Left-side padding
                cA[os] += lpd[l] * inp[is];
                cD[os] += hpd[l] * inp[is];
            }
            else if ((t - l) >= N)
            {
                is = (2 * N - t + l - 1) * istride;
                //printf("Right padding: t=%d, l=%d, is=%d, os=%d\n", t, l, is, os); // Debug: Right-side padding
                cA[os] += lpd[l] * inp[is];
                cD[os] += hpd[l] * inp[is];

            }
        }
    }
}


void dwt(wt_object wt, const SAMPLE_TYPE *input) {
    int J = wt->J;
    int temp_len = wt->siglength;
    int i; // will be reused many times
    COEFFICIENT_TYPE orig[temp_len];
    COEFFICIENT_TYPE orig2[temp_len];


    for (i = 0; i < wt->siglength; ++i)
    {
        orig[i] = input[i];
    }


    int N = wt->outlength; // N points to the end of the output buffer at first
    int iter;
    int len_cA;
    int lp = wt->wave->lpd_len;
    for (iter = 0; iter < J; ++iter)
    {
        len_cA = wt->length[J - iter];
        N -= len_cA;

        // address of detail coefficients is dwt_coeff + N because N is the latest end of approximation coefficients
        dwt_sym_stride(orig, temp_len, wt->wave->lpd, wt->wave->hpd, lp, orig2, len_cA, wt->dwt_coeff + N, 1, 1);
        temp_len = wt->length[J - iter];


        // orig hold original signal, orig2 holds cA which varies from iteration to iteration


        if (iter == J - 1)
        {
            for (i = 0; i < len_cA; ++i) // this only fills the last segment of dwt_coeff, the rest were filled with detail coeffs in prior iterations
            {
                wt->dwt_coeff[i] = orig2[i]; // the final round of cA is given to wt->dwt_coeff directly
            }
        }
        else
        {
            for (i = 0; i < len_cA; ++i)
            {
                orig[i] = orig2[i]; // copy cA back to orig for next iteration
            }
        }
    }

}


