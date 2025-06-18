/*
 * Riccardo Antonello (riccardo.antonello@unipd.it)
 * 
 * December 15, 2017
 *
 * Dept. of Information Engineering, University of Padova 
 *
 */

#ifndef COBS_H
#define COBS_H

#include <stdint.h>

/*  Function prototypes  */
void cobsEncode(const uint8_t *ptr, int length, uint8_t *dst);
void cobsDecode(const uint8_t *ptr, int length, uint8_t *dst);

#endif
