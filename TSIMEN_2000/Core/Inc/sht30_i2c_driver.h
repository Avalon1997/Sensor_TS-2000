#ifndef __SHT30_I2C_DRIVER
#define __SHT30_I2C_DRIVER

#include "I2C.h"
/* AADR Pin Connect to VSS*/
#define SHT30_ADDR_WRITE    0x44<<1         //10001000
#define SHT30_ADDR_READ     (0x44<<1)+1     //10001001

typedef enum
{
    /* 软件复位命令 */

    SOFT_RESET_CMD = 0x30A2,
    /*
    单次测量模式
    命名格式：Repeatability_CS_CMD
    CS: Clock stretching
    */
    HIGH_ENABLED_CMD    = 0x2C06,
    MEDIUM_ENABLED_CMD  = 0x2C0D,
    LOW_ENABLED_CMD     = 0x2C10,
    
    HIGH_DISABLED_CMD   = 0x2400,
    MEDIUM_DISABLED_CMD = 0x240B,
    LOW_DISABLED_CMD    = 0x2416,

    // /*
    // 周期测量模式
    // 命名格式：Repeatability_MPS_CMD
    // MPS：measurement per second
    //  */
    // HIGH_0_5_CMD    = 0x2032,
    // MEDIUM_0_5_CMD  = 0x2024,
    // LOW_0_5_CMD     = 0x202F,
    // HIGH_1_CMD      = 0x2130,
    // MEDIUM_1_CMD    = 0x2126,
    // LOW_1_CMD       = 0x212D,
    // HIGH_2_CMD      = 0x2236,
    // MEDIUM_2_CMD    = 0x2220,
    // LOW_2_CMD       = 0x222B,
    // HIGH_4_CMD      = 0x2334,
    // MEDIUM_4_CMD    = 0x2322,
    // LOW_4_CMD       = 0x2329,
    // HIGH_10_CMD     = 0x2737,
    // MEDIUM_10_CMD   = 0x2721,
    // LOW_10_CMD      = 0x272A,
    // /* 周期测量模式读取数据命令 */
    // READOUT_FOR_PERIODIC_MODE = 0xE000,
} SHT30_CMD;


uint8_t SHT30_ValGet (uint8_t* dat);
uint8_t SHT30_Dat_To_Float(uint8_t* const dat,float* temperature,float* humidity);
extern uint8_t I2CRXBuffer[6];

void Measure_TR(void);
void StringOut(char str,int a);


#endif  //end of __SHT30_I2C_DRIVER
