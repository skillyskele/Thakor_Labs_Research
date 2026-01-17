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
typedef float COMPRESSION_TYPE;



typedef struct {
    uint16_t idx;
    int16_t codeword;
} CodewordEntry;


#endif /* COMPRESSION_TYPES_H_ */
