#ifndef DWT_H
#define DWT_H

#include "wavedec.h"
#include "compression.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Discrete Wavelet Transform (symmetric extension) stride-based API.
    Matches the signature from dwt.c */
void dwt_sym_stride(COEFFICIENT_TYPE *inp, int N, COEFFICIENT_TYPE *lpd,  COEFFICIENT_TYPE *hpd, int lpd_len,
                           COEFFICIENT_TYPE *cA, int len_cA,  COEFFICIENT_TYPE *cD, int istride, int ostride);

void dwt(wt_object wt, const SAMPLE_TYPE *input);

#ifdef __cplusplus
}
#endif

#endif /* DWT_H */
