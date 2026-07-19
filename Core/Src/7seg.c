/*
 * 7seg.c
 *
 *  Common Anode (5161BS)
 */

#include "main.h"
#include "7seg.h"
#include "stm32f4xx_hal.h"

volatile uint8_t DisplayValue = 0;
volatile uint8_t pos = 0;

const uint8_t Mask[] =
{
    0b00111111, //0
    0b00000110, //1
    0b01011011, //2
    0b01001111, //3
    0b01100110, //4
    0b01101101, //5
    0b01111101, //6
    0b00000111, //7
    0b01111111, //8
    0b01101111  //9
};

void Set7SegDisplayValue(int val)
{
    if(val < 0)
        val = 0;

    if(val > 99)
        val = 99;

    DisplayValue = (uint8_t)val;
}

void Run7SegDisplay(void)
{
    uint8_t val;

    pos ^= 1;      // đổi LED mỗi lần gọi

    // Tắt cả hai LED trước
    HAL_GPIO_WritePin(PORT_7SEG_CONTROL0, PIN_7SEG_CONTROL0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PORT_7SEG_CONTROL1, PIN_7SEG_CONTROL1, GPIO_PIN_RESET);

    if(pos)
        val = Mask[DisplayValue % 10];
    else
        val = Mask[(DisplayValue / 10) % 10];

    /*********** Common Anode ***********/
    HAL_GPIO_WritePin(PORT_7SEG_P, PIN_7SEG_P,
            (val & 0x80) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    HAL_GPIO_WritePin(PORT_7SEG_G, PIN_7SEG_G,
            (val & 0x40) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    HAL_GPIO_WritePin(PORT_7SEG_F, PIN_7SEG_F,
            (val & 0x20) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    HAL_GPIO_WritePin(PORT_7SEG_E, PIN_7SEG_E,
            (val & 0x10) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    HAL_GPIO_WritePin(PORT_7SEG_D, PIN_7SEG_D,
            (val & 0x08) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    HAL_GPIO_WritePin(PORT_7SEG_C, PIN_7SEG_C,
            (val & 0x04) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    HAL_GPIO_WritePin(PORT_7SEG_B, PIN_7SEG_B,
            (val & 0x02) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    HAL_GPIO_WritePin(PORT_7SEG_A, PIN_7SEG_A,
            (val & 0x01) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    /************************************/

    // Bật LED cần hiển thị
    if(pos)
        HAL_GPIO_WritePin(PORT_7SEG_CONTROL0, PIN_7SEG_CONTROL0, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(PORT_7SEG_CONTROL1, PIN_7SEG_CONTROL1, GPIO_PIN_SET);
}
