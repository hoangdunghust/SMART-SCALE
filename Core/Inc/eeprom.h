#ifndef __EEPROM_H
#define __EEPROM_H

#include "main.h"

// Địa chỉ 7-bit là 0x50, dịch trái 1 bit là 0xA0
#define EEPROM_I2C_ADDR  0xA0

void EEPROM_Write(uint16_t mem_addr, uint8_t *data, uint16_t size);
void EEPROM_Read(uint16_t mem_addr, uint8_t *data, uint16_t size);
int Find_User(uint8_t *uid);
int Create_User(uint8_t *uid);

#endif
