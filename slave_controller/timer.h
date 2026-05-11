#ifndef GLOB_TIMER_H
#define GLOB_TIMER_H

#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif

/*timer function*/
void timerGlobInit(void);
uint16_t getTicks();
void updateGlobTimer();
void setUpdateFunction(void (*update_ptr)());
void setUpdateFunctionAndPeriod(void (*update_ptr)(), uint16_t period_ms);
void updateGlobTimer2();

void startTimerTick(int num);
uint32_t getTimerTick(int num);

#ifdef __cplusplus
}
#endif

#endif //GLOB_TIMER_H