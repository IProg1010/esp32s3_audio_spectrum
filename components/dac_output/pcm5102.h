#ifndef DAC_OUT_PCM5102
#define DAC_OUT_PCM5102

#include "stdint.h"


void initDAC();
void writeBuffer(uint16_t* data, uint16_t length);
void writezeroDAC();

#endif //DAC_OUT_PCM5102
