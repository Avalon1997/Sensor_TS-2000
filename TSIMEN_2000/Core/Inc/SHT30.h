#ifndef __SHT30_H_
#define __SHT30_H_

#include <stdbool.h>
#include <stdio.h>
#include "main.h"

#define SHT_ADDR    (0X44<<1)

uint8_t SHT30Init(void);
uint8_t SHT30Sample(void);

#endif
