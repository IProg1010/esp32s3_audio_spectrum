#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>


#ifdef __cplusplus
    extern "C" {
#endif

enum EncoderDir
{
    dirNot = 0,
    dirLeft = 1,
    dirRight = 2,
};

void initEncoder(void);
uint16_t getEncoderCnt();
uint16_t checkEncoderCnt();
uint8_t getSpeed();
enum EncoderDir getEncoderDirection();
void updateEncoder();

#ifdef __cplusplus
    }
#endif

#endif //ENCODER_H