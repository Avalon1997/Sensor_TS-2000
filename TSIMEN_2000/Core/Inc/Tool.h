#ifndef __TOOL__
#define __TOOL__

#include "main.h"

uint16_t ModBus_CRC16( uint8_t *pdata, int len);
void memv(uint8_t *pdata,uint8_t *ppdata,int st1,int st2,int length);
void Send_Double(double *pr,uint8_t a);
void Receive_Double(uint8_t *pr,int a);
void Send_Float(float *pr,int a);
void Receive_Float(uint8_t *pr,int a);

extern int CRC_j, 
    CRC_i,
    memv_i,
    Dou_i,
    Ene_i;

extern uint8_t DoubleBuf[8],
        FloatBuf[4];

extern uint16_t crc;

extern float Energy_Coefficient[2048];
extern double Wave_Coefficient[6];
















#endif
