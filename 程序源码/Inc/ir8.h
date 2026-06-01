#ifndef __IR8_H
#define __IR8_H

#include "mytype.h"

void ir8_init(void);
u8   ir8_read_bits(void);    /* returns 8-bit sensor bitmap, 1=line detected */

#endif
