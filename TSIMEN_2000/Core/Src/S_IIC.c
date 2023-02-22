#include "S_IIC.h"


#define SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(unsigned int)8<<28;}
#define SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(unsigned int)3<<28;}

uint8_t th[5];

// static void IIC_Delay()

/**
 * @brief Master send ACK signal
 * 
 */
void IIC_ACK(void)
{
    SDA_OUT();
    HAL_GPIO_WritePin(GPIOB,SCL_Pin,GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(GPIOB,SDA_Pin,GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(GPIOB,SCL_Pin,GPIO_PIN_SET);
    delay_us(2);
    HAL_GPIO_WritePin(GPIOB,SCL_Pin,GPIO_PIN_RESET);
    delay_us(1);
}

/**
 * @brief Master do not send ACK signal
 * 
 */
void IIC_NACK(void)
{
    SDA_OUT();
    HAL_GPIO_WritePin(GPIOB,SCL_Pin,GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(GPIOB,SDA_Pin,GPIO_PIN_SET);
    delay_us(2);
    HAL_GPIO_WritePin(GPIOB,SCL_Pin,GPIO_PIN_SET);
    delay_us(2);
    HAL_GPIO_WritePin(GPIOB,SCL_Pin,GPIO_PIN_RESET);
    delay_us(1);
}

/**
 * @brief Master wait for ACK from the low IC
 * 
 * @return uint8_t 
 */
uint8_t IIC_WaitACK(void)
{
    uint8_t t = 200;
    SDA_OUT();
    HAL_GPIO_WritePin(GPIOB,SDA_Pin,GPIO_PIN_SET);
    delay_us(1);
    HAL_GPIO_WritePin(GPIOB,SCL_Pin,GPIO_PIN_RESET); 
    delay_us(1);
    SDA_IN();
    delay_us(1);
    while (HAL_GPIO_ReadPin(GPIOB,SDA_Pin))
    {
        /* code */
        t --;
        delay_us(1);
        if (t==0)
        {
            HAL_GPIO_WritePin(GPIOB,SCL_Pin,GPIO_PIN_RESET);
            return 1;
        }
        delay_us(1);
    }
    delay_us(1);
    HAL_GPIO_WritePin(GPIOB,SCL_Pin,GPIO_PIN_SET);
    delay_us(1);
    HAL_GPIO_WritePin(GPIOB,SCL_Pin,GPIO_PIN_RESET);
    return 0;
}

/**
 * @brief IIC Start
 * 
 */
void IIC_Start(void)
{
    SDA_OUT();
    HAL_GPIO_WritePin(GPIOB,SDA_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,SCL_Pin,GPIO_PIN_SET);
    delay_us(4);
    HAL_GPIO_WritePin(GPIOB,SDA_Pin,GPIO_PIN_RESET);
    delay_us(4);
    HAL_GPIO_WritePin(GPIOB,SCL_Pin,GPIO_PIN_RESET);
}

/**
 * @brief IIC Stop
 * 
 */
void IIC_Stop(void)
{
    SDA_IN();
    HAL_GPIO_WritePin(GPIOB,SCL_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,SDA_Pin,GPIO_PIN_RESET);
    delay_us(4);
    HAL_GPIO_WritePin(GPIOB,SCL_Pin,GPIO_PIN_SET);
    delay_us(4);
    HAL_GPIO_WritePin(GPIOB,SDA_Pin,GPIO_PIN_SET);
    delay_us(4);
}

/**
 * @brief Send a byte
 * 
 * @param byte 
 */
void IIC_SendByte(uint8_t byte)
{
    uint8_t bitcnt;
    SDA_OUT();
    HAL_GPIO_WritePin(GPIOB,SCL_Pin,GPIO_PIN_RESET);
    for (bitcnt=0;bitcnt<8;bitcnt++)
    {
        if (byte&0x80) {HAL_GPIO_WritePin(GPIOB,SDA_Pin,GPIO_PIN_SET);}
        else {HAL_GPIO_WritePin(GPIOB,SDA_Pin,GPIO_PIN_RESET);}
        byte = byte << 1;
        delay_us(2);
        HAL_GPIO_WritePin(GPIOB,SCL_Pin,GPIO_PIN_SET);
        delay_us(2);
        HAL_GPIO_WritePin(GPIOB,SCL_Pin,GPIO_PIN_RESET);
        delay_us(2);
    }
}

/**
 * @brief Receive a byte
 * 
 * @return uint8_t 
 */
uint8_t IIC_ReceiveData(void)
{
    uint8_t rece;
    uint8_t bitcnt;
    rece = 0;
    SDA_IN();
    delay_us(1);
    for (bitcnt = 0; bitcnt < 8; bitcnt++)
    {
        /* code */
        HAL_GPIO_WritePin(GPIOB,SCL_Pin,GPIO_PIN_RESET);
        delay_us(2);
        HAL_GPIO_WritePin(GPIOB,SCL_Pin,GPIO_PIN_SET);
        rece = rece << 1;
        if (HAL_GPIO_ReadPin(GPIOB,SDA_Pin)) {rece = rece | 1;}
        delay_us(1);
    }
    HAL_GPIO_WritePin(GPIOB,SCL_Pin,GPIO_PIN_RESET);
    return rece;
}

/**
 * @brief read the sht30 data
 * 
 * @param addr 
 */
void SHT30_ReadData(uint8_t addr)
{
    uint16_t temp,humd;
    uint16_t buff[6];
    float Temperature = 0;
    float Humidity = 0;

    IIC_Start();
    IIC_SendByte(addr<<1 | RESET);
    IIC_WaitACK();
    IIC_SendByte(0x2C);
    IIC_WaitACK();
    IIC_SendByte(0x06);
    IIC_WaitACK();
    IIC_Stop();
    HAL_Delay(50);
    
    IIC_Start();
    IIC_SendByte(addr<<1 | SET);
    if( IIC_WaitACK() == RESET)
    {
        buff[0] = IIC_ReceiveData();
        IIC_ACK();
        buff[1] = IIC_ReceiveData();
        IIC_ACK();
        buff[2] = IIC_ReceiveData();
        IIC_ACK();
        buff[3] = IIC_ReceiveData();
        IIC_ACK();
        buff[4] = IIC_ReceiveData();
        IIC_ACK();
        buff[5] = IIC_ReceiveData();
        IIC_NACK();
        IIC_Stop();
    }

    // HAL_UART_Transmit(&huart1,(uint8_t*)buff,12,200);

    temp = ((buff[0]<<8) | buff[1]);
    humd = ((buff[3]<<8) | buff[4]);

    Temperature= (175.0*(float)temp/65535.0-45.0) ;     // T = -45 + 175 * tem / (2^16-1)
	Humidity= (100.0*(float)humd/65535.0);              // RH = hum*100 / (2^16-1)

    // sprintf(th,"%f",Temperature);
    // HAL_UART_Transmit(&huart1,th,sizeof(th),200);
    // sprintf(th,"%f",Humidity);
    // HAL_UART_Transmit(&huart1,th,sizeof(th),200);
    // printf("Temp: %f\r\nHumd: %f",Temperature,Humidity);
    printf("%.2f%.2f",Temperature,Humidity);

    temp = 0;
    humd = 0;
}










