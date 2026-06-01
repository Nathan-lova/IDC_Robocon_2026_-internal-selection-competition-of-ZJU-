#ifndef __IR8_H
#define __IR8_H

#include "mytype.h"

#define IR8_ADDR_7BIT  0x30
#define IR8_REG_SENSOR 0x50
#define IR8_CH_COUNT    8
#define IR8_CENTER      3.5f

typedef struct {
    u8     raw;
    u8     active_count;
    float  centroid;
    float  error;
    u8     valid;
} ir8_data_t;

void    ir8_init(void);
u8      ir8_read(ir8_data_t *data);

#endif
