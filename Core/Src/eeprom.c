#include "eeprom.h"
#include "user.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c1;

void EEPROM_Write(uint16_t mem_addr, uint8_t *data, uint16_t size) {
    HAL_I2C_Mem_Write(&hi2c1, EEPROM_I2C_ADDR, mem_addr, I2C_MEMADD_SIZE_16BIT, data, size, 100);
    HAL_Delay(5);
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
