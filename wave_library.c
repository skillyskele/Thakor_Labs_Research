#include "wave_objects.h"
#include "wave_library.h"
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
void dwt_sym_stride(double *inp, int N, double *lpd, double *hpd, int lpd_len, double *cA, int len_cA, double *cD, int istride, int ostride)
{
    int i, l, t, len_avg;
    int is, os;
    len_avg = lpd_len;

    for (i = 0; i < len_cA; ++i) // N is incorrect. N is 25. len_cA is correct
    {
        t = 2 * i + 1;
        os = i * ostride;
        cA[os] = 0.0;
        cD[os] = 0.0;
        printf("Debug: N=%d, len_cA=%d, len_avg=%d\n", N, len_cA, len_avg); // Debug: Show N, len_cA, and t values
        for (l = 0; l < len_avg; ++l)
        {
            if ((t - l) >= 0 && (t - l) < N)
            {
                is = (t - l) * istride;
                printf("Normal: t=%d, l=%d, is=%d, os=%d\n", t, l, is, os); // Debug: Normal access
                cA[os] += lpd[l] * inp[is];
                cD[os] += hpd[l] * inp[is];
            }
            else if ((t - l) < 0)
            {
                is = (-t + l - 1) * istride; //represents how you're growing into it. so t = 1. then take off 1 before you start reflecting, then let l grow naturally: 0, 1, 2, 3...
                printf("Left padding: t=%d, l=%d, is=%d, os=%d\n", t, l, is, os); // Debug: Left-side padding
                cA[os] += lpd[l] * inp[is];
                cD[os] += hpd[l] * inp[is];
            }
            else if ((t - l) >= N)
            {
                is = (2 * N - t + l - 1) * istride;
                printf("Right padding: t=%d, l=%d, is=%d, os=%d\n", t, l, is, os); // Debug: Right-side padding
                cA[os] += lpd[l] * inp[is];
                cD[os] += hpd[l] * inp[is];

            }
        }
    }
}

void dwt(wt_object wt, const double *input) {
    int J = wt->J;
    int temp_len = wt->siglength;
    int i; // will be reused many times
    wt->length[J+1] = temp_len;
    wt->outlength = 0;
    double *orig = (double *)malloc(sizeof(double) * temp_len); // we can avoid malloc if we decide beforehand how large to make siglength
    double *orig2 = (double *)malloc(sizeof(double) * temp_len);

    for (i = 0; i < wt->siglength; ++i)
    {
        orig[i] = input[i];
    }

    int N = temp_len;
    int lp = wt->wave->lpd_len;
    // so N, temp_len, are all just the length of the input signal

    i = J;
    while (i >0) {
        N = N + lp - 2; // example padding: [1 2 3 4] and [1 1] calls for padding like, [1 1 2 3 4 4] just lp - 1 each side
        N = (int) ceil((double) N / 2.0); // it'll downsample by 2 every level
        wt->length[i] = N;
        wt->outlength += wt->length[i]; // keep track of the end
        i--;
    }
    wt->length[0] = wt->length[1];
    /*
    length[0] = copy length 1, so we can hold the final approximation coefficients
    length[1] = decimated a lot
    length[2] =
    ...
    length[J] = decimated by 2 (after padding)
    length[J + 1] = input signal length
    */
    wt->outlength += wt->length[0];
    N = wt->outlength; // N points to the end of the output buffer, note that N is being REPURPOSED here

    int iter;
    int len_cA;
    for (iter = 0; iter < J; ++iter)
    {
        len_cA = wt->length[J - iter];
        N -= len_cA;

        // address of detail coefficients is dwt_coeff + N because N is the latest end of approximation coefficients
        dwt_sym_stride(orig, temp_len, wt->wave->lpd, wt->wave->hpd, lp, orig2, len_cA, wt->dwt_coeff + N, 1, 1);
        temp_len = wt->length[J - iter];

        if (iter == J - 1)
        {
            for (i = 0; i < len_cA; ++i)
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
    free(orig);
    free(orig2);
}

/*
Input data: EEG, ECG
test 1 second

*/
