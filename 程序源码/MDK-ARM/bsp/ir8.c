#include "ir8.h"
#include "stm32f4xx.h"

/* PF1 = SCL, PF0 = SDA */
#define IR8_SCL_PIN  GPIO_PIN_1
#define IR8_SDA_PIN  GPIO_PIN_0
#define IR8_PORT     GPIOF

#define SCL_H()  (IR8_PORT->BSRR = IR8_SCL_PIN)
#define SCL_L()  (IR8_PORT->BSRR = (uint32_t)IR8_SCL_PIN << 16)
#define SDA_H()  (IR8_PORT->BSRR = IR8_SDA_PIN)
#define SDA_L()  (IR8_PORT->BSRR = (uint32_t)IR8_SDA_PIN << 16)
#define SDA_R()  ((IR8_PORT->IDR & IR8_SDA_PIN) ? 1 : 0)

static void sda_out(void)
{
    IR8_PORT->MODER = (IR8_PORT->MODER & ~(3u << 0)) | (1u << 0);
}

static void sda_in(void)
{
    IR8_PORT->MODER &= ~(3u << 0);
}

static void delay_us(u16 n)
{
    u32 start = DWT->CYCCNT;
    u32 ticks = (u32)n * 168;
    while ((DWT->CYCCNT - start) < ticks);
}

static void i2c_start(void)
{
    sda_out();
    SDA_H();
    delay_us(5);
    SCL_H();
    delay_us(5);
    SDA_L();
    delay_us(5);
    SCL_L();
    delay_us(5);
}

static void i2c_stop(void)
{
    sda_out();
    SDA_L();
    delay_us(5);
    SCL_H();
    delay_us(5);
    SDA_H();
    delay_us(5);
}

static u8 i2c_write_byte(u8 data)
{
    u8 i;
    sda_out();
    for (i = 0; i < 8; i++) {
        if (data & 0x80) SDA_H(); else SDA_L();
        data <<= 1;
        delay_us(3);
        SCL_H();
        delay_us(5);
        SCL_L();
        delay_us(3);
    }
    /* read ACK */
    sda_in();
    SDA_H();
    delay_us(3);
    SCL_H();
    delay_us(5);
    u8 ack = SDA_R();
    SCL_L();
    delay_us(3);
    sda_out();
    return ack;
}

static u8 i2c_read_byte(u8 ack)
{
    u8 i, data = 0;
    sda_in();
    SDA_H();
    for (i = 0; i < 8; i++) {
        delay_us(3);
        SCL_H();
        delay_us(5);
        data = (data << 1) | SDA_R();
        SCL_L();
    }
    /* send ACK (0) or NACK (1) */
    sda_out();
    if (ack) SDA_H(); else SDA_L();
    delay_us(3);
    SCL_H();
    delay_us(5);
    SCL_L();
    delay_us(3);
    sda_in();
    SDA_H();
    return data;
}

void ir8_init(void)
{
    /* DWT: enable if not already (ps2_init enables it, but be defensive) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    __HAL_RCC_GPIOF_CLK_ENABLE();

    /* PF1 (SCL): output push-pull, high speed, no pull */
    GPIOF->MODER   = (GPIOF->MODER & ~(3u << 2)) | (1u << 2);
    GPIOF->OTYPER  &= ~(1u << 1);
    GPIOF->PUPDR   &= ~(3u << 2);
    GPIOF->OSPEEDR = (GPIOF->OSPEEDR & ~(3u << 2)) | (3u << 2);

    /* PF0 (SDA): output open-drain, no pull */
    GPIOF->MODER   = (GPIOF->MODER & ~(3u << 0)) | (1u << 0);
    GPIOF->OTYPER  |= (1u << 0);
    GPIOF->PUPDR   &= ~(3u << 0);
    GPIOF->OSPEEDR = (GPIOF->OSPEEDR & ~(3u << 0)) | (3u << 0);

    /* idle: both lines HIGH */
    SCL_H();
    SDA_H();
}

u8 ir8_read(ir8_data_t *data)
{
    u8 bits, ack;

    __set_BASEPRI(0x10);

    i2c_start();
    ack = i2c_write_byte((u8)(IR8_ADDR_7BIT << 1));       /* W */
    if (ack) goto fail;
    ack = i2c_write_byte(IR8_REG_SENSOR);
    if (ack) goto fail;

    i2c_start();                                            /* repeated start */
    ack = i2c_write_byte((u8)((IR8_ADDR_7BIT << 1) | 1));  /* R */
    if (ack) goto fail;

    bits = i2c_read_byte(1);   /* NACK on last byte */

    i2c_stop();
    __set_BASEPRI(0);

    /* compute centroid and error */
    data->raw = bits;
    data->active_count = 0;
    {
        u8 i;
        float sum = 0.0f;
        for (i = 0; i < 8; i++) {
            if (bits & (1 << i)) {
                data->active_count++;
                sum += (float)i;
            }
        }
        if (data->active_count > 0)
            data->centroid = sum / (float)data->active_count;
        else
            data->centroid = IR8_CENTER;
    }
    data->error  = data->centroid - IR8_CENTER;
    data->valid  = 1;
    return 1;

fail:
    i2c_stop();
    __set_BASEPRI(0);
    data->valid = 0;
    return 0;
}
