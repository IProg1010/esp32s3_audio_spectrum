#ifndef DELAY_H
#define DELAY_H

#include "ch32v20x_tim.h"
#include <stdint.h>

#ifdef __cplusplus
    extern "C" {
#endif

void delayInit();
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);

#ifdef __cplusplus
    }
#endif


#endif