#ifndef ADC_IN_PCM1804
#define ADC_IN_PCM1804

#include "stdint.h"

void initADC();
void readBuffer(uint16_t* data, uint16_t length);

#endif //ADC_IN_PCM1804
