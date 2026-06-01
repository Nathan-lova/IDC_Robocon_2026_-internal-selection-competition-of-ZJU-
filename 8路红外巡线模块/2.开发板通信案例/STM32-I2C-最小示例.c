#include "main.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;

#define IR8_ADDR_7BIT  (0x30)
#define IR8_ADDR_8BIT  (IR8_ADDR_7BIT << 1)
#define IR8_REG_SENSOR (0x50)

void IR8_ReadBits_Once(void)
{
    uint8_t bits = 0;
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Mem_Read(&hi2c1,
                           IR8_ADDR_8BIT,
                           IR8_REG_SENSOR,
                           I2C_MEMADD_SIZE_8BIT,
                           &bits,
                           1,
                           100);
    if (ret != HAL_OK) {
        printf("IR8 I2C read failed\r\n");
        return;
    }

    printf("IR8 bits=0x%02X\r\n", bits);
}

