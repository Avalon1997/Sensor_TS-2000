#include "sht30_i2c_driver.h"
#include <stdio.h>
#include "usart.h"
//-----SHT30功能函数-----
#define CRC8_POLYNOMIAL 0x31

uint8_t I2CRXBuffer[6];
uint8_t Error = 0;
uint8_t ErrorMessage[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
float Temperature = 0,Humidity = 0;

uint8_t intbuffer[4];

char th[5];

uint8_t Status;



/**
 * @brief   向SHT30发送一条指令（16bit)
 * @param   cmd —— SHT30指令 （在SHT30_MODE中枚举定义）
 * @retval  成功 —— 返回HAL_OK
 */

 static uint8_t SHT30_Send_CMD(SHT30_CMD cmd)
 {
     uint8_t cmd_buffer[2];
     cmd_buffer[0] = cmd >> 8;
     cmd_buffer[1] = cmd;
     Status = HAL_I2C_Master_Transmit(&hi2c1,SHT30_ADDR_WRITE,(uint8_t*) cmd_buffer,2,0xFFFF);
     printf("%d",Status);
     return Status;

     
 }


/**
 * @brief  单次测量获取数据
 * @param  dat —— 存储读取数据的地址 （6个字节数组）
 * @retval 成功 —— 返回HAL_OK 
 */
uint8_t SHT30_ValGet(uint8_t* dat)
{
    uint8_t Error;
    Error = SHT30_Send_CMD(HIGH_ENABLED_CMD);
    HAL_Delay(25);
    if (Error != HAL_OK)
    {
        return Error;
    }

    return HAL_I2C_Master_Receive(&hi2c1,SHT30_ADDR_READ,dat,6,0xFFFF);
}



/**
 * @brief CRC校验函数
 * 
 * @param message 
 * @param initial_value 
 * @return uint8_t 
 */
uint8_t CheckCrc8(uint8_t* const message,uint8_t initial_value)
{
    uint8_t remainder;      //余数
    uint8_t i = 0,j = 0;    //循环变量

    /* 初始化 */
    remainder = initial_value;

    for(j = 0;j < 2;j++)
    {
        remainder ^= message[j];

        /* 从最高位开始依次计算 */
        for (i = 0;i < 8;i++)
        {
            if (remainder & 0x80)
            {
                remainder = (remainder << 1)^CRC8_POLYNOMIAL;
            }
            else 
            {
                remainder = (remainder << 1);
            }
        }
    }
    
    /* 返回计算的CRC校验码 */
    return remainder;
}


/**
 * @brief   将SHT30接收的6个字节数据进行CRC校验，并转换为温度值和湿度值
 * @param   dat —— 存储接收数据的地址（6个字节数组）
 * @retval  校验成功 —— 返回0；校验失败 —— 返回1，并设置温度值和湿度值为0
 */
uint8_t SHT30_Dat_To_Float(uint8_t* const dat,float* temperature,float* humidity)
{
    /* 初始化温度湿度值 */
    uint16_t recv_temperature = 0;
    uint16_t recv_humidity = 0;

    /* 校验温度数据和湿度数据是否接收正确 */
    if(CheckCrc8(dat,0xFF) != dat[2] || CheckCrc8(&dat[3],0xFF) !=dat[5])
    {
        return 1;
    }

    /* 转换温度数据 */
    recv_temperature = ((uint16_t)dat[0]<<8)|dat[1];
    *temperature = -45+175*((float)recv_temperature/65536);

    /* 转换湿度数据 */
    recv_humidity = ((uint16_t)dat[3]<<8)|dat[4];
    *humidity = 100 * ((float)recv_humidity/65536);

    return 0;

}


/**
 * @brief 执行测量，串口传输数据
 * 
 */
void Measure_TR(void)
{
    int j;
    //    for(j=0;j<100;j++)
    // {   
        // if(SHT30_ValGet(I2CRXBuffer) == HAL_OK)
        // {
        //     Error = SHT30_Dat_To_Float(I2CRXBuffer,&Temperature,&Humidity);
        //     if (!Error)
        //     {
                
        //         // StringOut('a',1);
                
        //         sprintf(th,"%f",Temperature);
        //         HAL_UART_Transmit(&huart1,(uint8_t *)th,5,0xFFFF);
        //         sprintf(th,"%f",Humidity);
        //         HAL_UART_Transmit(&huart1,(uint8_t *)th,5,0xFFFF);

        //         //  printf("Temperature:%.2fC,Humidity:%.2f%%\r\n",Temperature,Humidity);
        //         //  printf("%.2f%.2f",Temperature,Humidity);
        //          break;
        //     }
        
        //     else
        //     {
        //         printf("CRC Error\r\n");
        //         break;
        //     }
        // }
    // }
    // if (j == 100)
    // {
    //     // printf("SHT30_ValGet Error\r\n");
    //     // printf("Please check on the equipment\r\n");
    //     HAL_UART_Transmit(&huart1,ErrorMessage,8,0xFFFF);
    //     j = 0;
    // }

    j = SHT30_ValGet(I2CRXBuffer);
    if (j == HAL_OK)
    {
        Error = SHT30_Dat_To_Float(I2CRXBuffer,&Temperature,&Humidity);
        if (!Error)
        {
            
            // StringOut('a',1);
            
            // sprintf(th,"%f",Temperature);
            // HAL_UART_Transmit(&huart1,(uint8_t *)th,5,0xFFFF);
            // sprintf(th,"%f",Humidity);
            // HAL_UART_Transmit(&huart1,(uint8_t *)th,5,0xFFFF);

             printf("Temperature:%.2fC,Humidity:%.2f%%\r\n",Temperature,Humidity);
             printf("%.2f%.2f",Temperature,Humidity);
                // break;
        }

        else
        {
            printf("CRC Error\r\n");
            // break;
        } 
    }
    else printf("j = %d",j);
    
}

void StringOut(char str,int a)
{
    if(str == 'a')
    {
        HAL_UART_Transmit(&huart1,"Hello!!!",8,0xFFFF);
    }
    else if (str == 'b')
    {
        // HAL_UART_Transmit(&huart2,"Hello!!!",8,0xFFFF);
    }

}





