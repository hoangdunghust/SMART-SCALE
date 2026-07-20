// Force recompile: size update for UserProfile_t
#include "eeprom.h"
#include "user.h"
#include "history.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c1;

void EEPROM_Write(uint16_t mem_addr, uint8_t *data, uint16_t size) {
    uint16_t page_size = 8; // An toàn cho mọi loại EEPROM (24C02, 24C32...)
    uint16_t bytes_written = 0;
    
    while (bytes_written < size) {
        uint16_t addr = mem_addr + bytes_written;
        uint16_t max_write = page_size - (addr % page_size);
        uint16_t chunk_size = size - bytes_written;
        if (chunk_size > max_write) {
            chunk_size = max_write;
        }
        
        HAL_I2C_Mem_Write(&hi2c1, EEPROM_I2C_ADDR, addr, I2C_MEMADD_SIZE_16BIT, &data[bytes_written], chunk_size, 100);
        HAL_Delay(5); // Chờ ghi xong
        
        bytes_written += chunk_size;
    }
}

void EEPROM_Read(uint16_t mem_addr, uint8_t *data, uint16_t size) {
    HAL_I2C_Mem_Read(&hi2c1, EEPROM_I2C_ADDR, mem_addr, I2C_MEMADD_SIZE_16BIT, data, size, 100);
}
int Find_User(uint8_t *uid)
{

    UserProfile_t user;


    for(int i=0;i<MAX_USER;i++)
    {

        EEPROM_Read(
            USER_ADDR(i),
            (uint8_t*)&user,
            sizeof(UserProfile_t)
        );


        if(memcmp(user.uid,uid,4)==0)
        {
            return i;
        }

    }


    return -1;
}
int Create_User(uint8_t *uid)
{

    UserProfile_t user;


    for(int i=0;i<MAX_USER;i++)
    {


        EEPROM_Read(
            USER_ADDR(i),
            (uint8_t*)&user,
            sizeof(UserProfile_t)
        );


        // vùng trống
        if(user.uid[0]==0xFF)
        {


            memset(&user,0,sizeof(user));


            memcpy(
                user.uid,
                uid,
                4
            );


            user.count=0;


            EEPROM_Write(
                USER_ADDR(i),
                (uint8_t*)&user,
                sizeof(user)
            );


            return i;

        }

    }


    return -1;
}

#define EEPROM_INIT_FLAG_ADDR 0xFE
#define EEPROM_MAGIC_VAL      0xAB

void EEPROM_Init(void)
{
    uint8_t flag = 0;
    EEPROM_Read(EEPROM_INIT_FLAG_ADDR, &flag, 1);
    
    if (flag != EEPROM_MAGIC_VAL)
    {
        uint8_t erase_buf[8];
        memset(erase_buf, 0xFF, sizeof(erase_buf));
        
        // Xóa 176 bytes dữ liệu user (từ 0 đến 175)
        for (uint16_t addr = 0; addr < 176; addr += 8)
        {
            EEPROM_Write(addr, erase_buf, 8);
        }
        
        // Khởi tạo count lịch sử hệ thống về 0
        uint8_t zero = 0;
        EEPROM_Write(COUNT_ADDR, &zero, 1);
        
        // Ghi cờ đã cấu hình
        uint8_t magic = EEPROM_MAGIC_VAL;
        EEPROM_Write(EEPROM_INIT_FLAG_ADDR, &magic, 1);
    }
}
