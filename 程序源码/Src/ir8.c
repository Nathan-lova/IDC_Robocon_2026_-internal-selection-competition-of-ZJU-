#include "ir8.h"
#include "stm32f4xx_hal.h"

/*
 * Software I2C on PD14 (SCL) / PD15 (SDA) for 8-ch IR line sensor.
 * Sensor address: 7-bit 0x30, register 0x50 = 8-bit sensor bitmap.
 * Standard-mode timing (~100kHz), tuned for 168MHz SYSCLK.
 */

#define IR8_SCL   GPIO_PIN_14
#define IR8_SDA   GPIO_PIN_15
#define IR8_PORT  GPIOD
#define IR8_ADDR  ((u8)0x30)

#define SCL_H()   (IR8_PORT->BSRR = IR8_SCL)
#define SCL_L()   (IR8_PORT->BSRR = (uint32_t)IR8_SCL << 16)
#define SDA_H()   (IR8_PORT->BSRR = IR8_SDA)
#define SDA_L()   (IR8_PORT->BSRR = (uint32_t)IR8_SDA << 16)
#define SDA_R()   ((IR8_PORT->IDR & IR8_SDA) ? 1 : 0)

static void sda_out(void)
{
    IR8_PORT->MODER  = (IR8_PORT->MODER & ~(3u << 30)) | (1u << 30);
    IR8_PORT->OTYPER &= ~(1u << 15);
}

static void sda_in(void)
{
    IR8_PORT->MODER  =  IR8_PORT->MODER & ~(3u << 30);
    IR8_PORT->PUPDR  = (IR8_PORT->PUPDR & ~(3u << 30)) | (1u << 30);  /* pull-up */
}

static void delay_5us(void)
{
    volatile u32 i;
    for (i = 0; i < 70; i++) { __NOP(); }
}

static void i2c_start(void)
{
    sda_out();
    SDA_H();  SCL_H();  delay_5us();
    SDA_L();              delay_5us();
    SCL_L();
}

static void i2c_stop(void)
{
    sda_out();
    SDA_L();              delay_5us();
    SCL_H();              delay_5us();
    SDA_H();
}

static u8 i2c_write_byte(u8 data)
{
    u8 i;
    sda_out();
    for (i = 0; i < 8; i++) {
        if (data & 0x80) SDA_H(); else SDA_L();
        delay_5us();
        SCL_H();  delay_5us();
        SCL_L();
        data <<= 1;
    }
    sda_in();
    SCL_H();  delay_5us();
    i = SDA_R() ? 0 : 1;    /* 1 = ACK, 0 = NACK */
    SCL_L();
    return i;
}

static u8 i2c_read_byte(u8 ack)
{
    u8 i, data = 0;
    sda_in();
    for (i = 0; i < 8; i++) {
        SCL_H();  delay_5us();
        data = (u8)((data << 1) | SDA_R());
        SCL_L();  delay_5us();
    }
    sda_out();
    if (ack) SDA_L(); else SDA_H();
    delay_5us();
    SCL_H();  delay_5us();
    SCL_L();
    return data;
}

void ir8_init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /* PD14/PD15: output push-pull, idle HIGH */
    GPIOD->MODER  = (GPIOD->MODER & ~((3u << 28) | (3u << 30))) | ((1u << 28) | (1u << 30));
    GPIOD->OTYPER &= ~((1u << 14) | (1u << 15));
    SCL_H();
    SDA_H();
}

u8 ir8_read_bits(void)
{
    u8 bits = 0;

    i2c_start();
    if (!i2c_write_byte((u8)(IR8_ADDR << 1))) {   /* write address */
        i2c_stop();
        return 0x00;   /* NACK = sensor absent → no line, safe stop */
    }
    if (!i2c_write_byte(0x50)) {                   /* register address */
        i2c_stop();
        return 0x00;
    }
    i2c_start();
    if (!i2c_write_byte((u8)((IR8_ADDR << 1) | 1))) { /* read address */
        i2c_stop();
        return 0x00;
    }
    bits = i2c_read_byte(0);                       /* NACK after last byte */
    i2c_stop();
    return bits;
}
