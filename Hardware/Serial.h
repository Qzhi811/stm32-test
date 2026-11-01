#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdint.h>
#include <stdio.h>

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendString(char *String);
uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);

#endif
