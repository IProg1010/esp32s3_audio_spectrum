#ifndef DI_DO_CONTROL_H
#define DI_DO_CONTROL_H

#include <stdint.h>
#include "ch32v30x_gpio.h"


#ifdef __cplusplus
    extern "C" {
#endif

enum PinState
{
    stOn = 1,
    stOff = 0,
    stUnknow = -1,
};

enum PinType
{
    tInput = 1,
    tOutput = 0,
};

enum InMode
{
	iFL,
  	iPD,
  	iPU,
    InUnknow,
}; 

enum OutMode
{
    oOD,
    oPP,
    OutUnknow,
};

enum Port
{
	pA,
  	pB,
  	pC,
  	pD,
  	pE,
}; 


enum PinActive
{
    actOn = 1,
    actOff = 0,
};

enum OutType
{
    otNone = 0,
    otBlink = 1,
    otDurationBlink = 2,
    otSet = 3,
    otDurationSet = 4,
    otInverse = 5,
    otDurationInverse = 6,
    otImpulse = 7,
    otDurationImpulse = 8,
};

enum InFilter
{
    fNone = 0,
    fOn = 1,
};

void initDiscreteIO(void);

void setIOpin(uint16_t pin, enum Port, uint8_t ch_num, enum PinType type, enum InMode in_type, enum OutMode out_type);
void resetIOpin(uint8_t ch_num);

uint8_t getInputState(uint8_t channel_num);
enum PinState getInputCurrentState(uint8_t channel_num);
enum PinState getInputCurrentStateQuickly(uint8_t channel_num);
void setInputFilter(uint8_t channel_num, enum InFilter filt, uint8_t filter_cnt);

//void setOutputState(uint8_t channel_num, uint8_t state);
void setOutputStateQuickly(uint8_t channel_num, enum PinState state);
void setOutputState(uint8_t channel_num, enum PinState state);
void setOutputStateForTime(uint8_t channel_num, enum PinState state, uint16_t period_ms);

enum PinState getCurrentOutputState(uint8_t channel_num);

void inverseOutputStateQuickly(uint8_t channel_num);
void inverseOutputState(uint8_t channel_num);
void inverseOutputStateForTime(uint8_t channel_num, uint16_t period_ms);

void blinkOutputStateQuickly(uint8_t channel_num, uint16_t period_ms);
void blinkOutputState(uint8_t channel_num, uint16_t period_ms);
void blinkOutputStateForDuration(uint8_t channel_num, uint16_t period_ms, uint16_t duration_ms, enum PinState end_state);

void impulseOutputStateQuickly(uint8_t channel_num, uint16_t period_ms);
void impulseOutputState(uint8_t channel_num, uint16_t period_ms);
void impulseOutputStateForTime(uint8_t channel_num, uint16_t period_ms, uint16_t duration_ms);

void updateForTimerMs();

#ifdef __cplusplus
    }
#endif

#endif //DI_DO_CONTROL_H