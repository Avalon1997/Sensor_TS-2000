# include "main.h"
# include "usart.h"
# include "Tool.h"

int CRC_j = 0, 
    CRC_i = 0,
    memv_i = 0,
    Dou_i = 0,
    Ene_i = 0;

uint8_t DoubleBuf[8],
        FloatBuf[4];

uint16_t crc = 0xFFFF;

float Energy_Coefficient[2048];
double Wave_Coefficient[6];

/* ---------------------------------------------------------------------- Tool Function -------------------------------------------------------------------------- */
/**
 * @brief CRC16 check code. The function is used for data verification when communicating with PC.
 *  
 * @param pdata 
 * @param len 
 * @return uint16_t 
 */
uint16_t ModBus_CRC16( uint8_t *pdata, int len)     //polynomial: 8005
{
  crc = 0xFFFF;
  for ( CRC_j=0; CRC_j<len;CRC_j++)
  {
    // printf("%d",crc);
    // HAL_Delay(20);
    crc=crc^pdata[CRC_j];
    for ( CRC_i=0; CRC_i<8; CRC_i++)
    {
      if( ( crc&0x0001) >0)
      {
        crc=crc>>1;
        crc=crc^ 0xA001;                            //We take 8005 in reverse.
      }
    else
    crc=crc>>1;
    }
  }
return crc;
}

/**
 * @brief Cut a specific length from the A array to the B array.
 * 
 * @param pdata target array
 * @param ppdata truncated array
 * @param st1 start bit of the destination array
 * @param st2 start bit of truncated array
 * @param length The length of the truncated array
 */
void memv(uint8_t *pdata,uint8_t *ppdata,int st1,int st2,int length)
{
  for (memv_i=0;memv_i<length;memv_i++)
  {
    pdata[memv_i+st1-1] = ppdata[memv_i+st2-1];
  }
}

/**
 * @brief Send Double array data
 * 
 * @param pr data array
 * @param a length
 */
void Send_Double(double *pr,uint8_t a)
{
  for (Dou_i=0;Dou_i<a;Dou_i++)
  {
  DoubleBuf[7] = *((uint8_t*)&pr[Dou_i]);
  DoubleBuf[6] = *((uint8_t*)&pr[Dou_i]+1);
  DoubleBuf[5] = *((uint8_t*)&pr[Dou_i]+2);
  DoubleBuf[4] = *((uint8_t*)&pr[Dou_i]+3);
  DoubleBuf[3] = *((uint8_t*)&pr[Dou_i]+4);
  DoubleBuf[2] = *((uint8_t*)&pr[Dou_i]+5);
  DoubleBuf[1] = *((uint8_t*)&pr[Dou_i]+6);
  DoubleBuf[0] = *((uint8_t*)&pr[Dou_i]+7);
  HAL_UART_Transmit(&huart1,DoubleBuf,sizeof(DoubleBuf),0xFFFF);
  // printf("%.15lf\r\n",Wave_Coefficient[Dou_i]);
  }
}

/**
 * @brief Receive double array data
 * 
 * @param pr data array
 * @param a length
 */
void Receive_Double(uint8_t *pr,int a)
{
  for (Dou_i=0;Dou_i<a;Dou_i ++)
  {
  DoubleBuf[0] = pr[9+Dou_i*8];
  DoubleBuf[1] = pr[8+Dou_i*8];
  DoubleBuf[2] = pr[7+Dou_i*8];
  DoubleBuf[3] = pr[6+Dou_i*8];
  DoubleBuf[4] = pr[5+Dou_i*8];
  DoubleBuf[5] = pr[4+Dou_i*8];
  DoubleBuf[6] = pr[3+Dou_i*8];
  DoubleBuf[7] = pr[2+Dou_i*8];
  Wave_Coefficient[Dou_i] = *((double*)&DoubleBuf[0]);
  // printf("%lf",Wave_Coefficient[Dou_i]);
  }
}

/**
 * @brief Send float array data
 * 
 * @param pr float data array
 * @param a length
 */
void Send_Float(float *pr,int a)
{
    for (Dou_i=0;Dou_i<a;Dou_i++)
  {
  FloatBuf[3] = *((uint8_t*)&pr[Dou_i]);
  FloatBuf[2] = *((uint8_t*)&pr[Dou_i]+1);
  FloatBuf[1] = *((uint8_t*)&pr[Dou_i]+2);
  FloatBuf[0] = *((uint8_t*)&pr[Dou_i]+3);
  HAL_UART_Transmit(&huart1,FloatBuf,sizeof(FloatBuf),0xFFFF);
  // printf("%.6f %d\r\n",pr[Dou_i],Dou_i);
  }
}

/**
 * @brief Receive float array data
 * 
 * @param pr float data array
 * @param a length
 */
void Receive_Float(uint8_t *pr,int a)
{
  if (a == 0x01)
  {
      for (Ene_i=0;Ene_i<1024;Ene_i++)
      {
        FloatBuf[0] = pr[7+Ene_i*4];
        FloatBuf[1] = pr[6+Ene_i*4];
        FloatBuf[2] = pr[5+Ene_i*4];
        FloatBuf[3] = pr[4+Ene_i*4];
        Energy_Coefficient[Ene_i*2] = *((float*)&FloatBuf[0]);
        // printf("%.6f %d\r\n",Energy_Coefficient[Ene_i*2],Ene_i*2);
      }
  }
  else if (a == 0x02)
  {
    for (Ene_i=0;Ene_i<1024;Ene_i++)
    {
        FloatBuf[0] = pr[7+Ene_i*4];
        FloatBuf[1] = pr[6+Ene_i*4];
        FloatBuf[2] = pr[5+Ene_i*4];
        FloatBuf[3] = pr[4+Ene_i*4];
        Energy_Coefficient[Ene_i*2+1] = *((float*)&FloatBuf[0]);
        // printf("%.6f %d\r\n",Energy_Coefficient[Ene_i*2+1],Ene_i*2+1);
    }
  }
  else 
  {
    HAL_UART_Transmit(&huart1,USART1_Tsimen_ERROR,sizeof(USART1_Tsimen_ERROR),0xFFFF);
  }
}

