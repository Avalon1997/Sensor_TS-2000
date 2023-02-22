#ifndef __S_IIC__
#define __S_IIC__

#include "main.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

void IIC_ACK(void);
void IIC_NACK(void);
uint8_t IIC_WaitACK(void);
void IIC_Start(void);
void IIC_Stop(void);
void IIC_SendByte(uint8_t byte);
uint8_t IIC_ReceiveData(void);
void SHT30_ReadData(uint8_t addr);

#endif




