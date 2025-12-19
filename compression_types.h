/*
 * compression_types.h
 *
 *  Created on: Dec 18, 2025
 *      Author: natha
 */

#ifndef COMPRESSION_TYPES_H_
#define COMPRESSION_TYPES_H_
#include <stdint.h>



typedef float COEFFICIENT_TYPE;
typedef uint16_t SAMPLE_TYPE; // will actually be uint16 with the iadc, but for testing, it's easier to have it float


typedef struct
{
    int16_t idx;
    int32_t codeword;
} CodewordEntry;


#endif /* COMPRESSION_TYPES_H_ */
