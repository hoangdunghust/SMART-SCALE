#include "ds1307.h"
#include <stdio.h>
extern I2C_HandleTypeDef hi2c1;

static uint8_t BCD2DEC(uint8_t val)
{
    return ((val>>4)*10)+(val&0x0F);
}

static uint8_t DEC2BCD(uint8_t val)
{
    return ((val/10)<<4)|(val%10);
}

void DS1307_GetTime(DS1307_Time *time)
{
    uint8_t buf[7];
    HAL_I2C_Mem_Read(&hi2c1, DS1307_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, buf, 7, 100);

    time->second = BCD2DEC(buf[0] & 0x7F);
    time->minute = BCD2DEC(buf[1]);
    time->hour   = BCD2DEC(buf[2] & 0x3F); // Đảm bảo lọc bit 12/24h nếu có
    time->day    = BCD2DEC(buf[3]);        // Thứ (Sun-Sat)
    time->date   = BCD2DEC(buf[4]);        // Ngày
    time->month  = BCD2DEC(buf[5]);        // Tháng
    time->year   = BCD2DEC(buf[6]);        // Năm
}

void DS1307_SetTime(DS1307_Time *time)
{
    uint8_t buf[7];

    buf[0] = DEC2BCD(time->second) & 0x7F; // CHẾ ĐỘ CHẠY (Bit 7 = 0)
    buf[1] = DEC2BCD(time->minute);
    buf[2] = DEC2BCD(time->hour);
    buf[3] = DEC2BCD(time->day);
    buf[4] = DEC2BCD(time->date);
    buf[5] = DEC2BCD(time->month);
    buf[6] = DEC2BCD(time->year);

    HAL_I2C_Mem_Write(&hi2c1, DS1307_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, buf, 7, 100);
}
