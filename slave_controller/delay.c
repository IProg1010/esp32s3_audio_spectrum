#include <delay.h>
#include <stdint.h>

static uint32_t count = 0;

void TIM2_IRQHandler(void) {
    __asm volatile ("call TIM2_IRQHandler_real; mret");
}


__attribute__((used)) void TIM2_IRQHandler_real(void) 
{
    //if(TIM2-> == SET )
    {
		count++;
    }
}


void delayTimerInit();
void delayInit(void)
{
    delayTimerInit();
}

void delay_ms(uint32_t ms)
{
    while(ms > 0)
    {
        delay_us(1000);
        ms--;
    }
}

void delay_us(uint32_t us)
{
    TIM2->CNT = 0;
    TIM_Cmd (TIM2, ENABLE);

    while(us > 0)
    {
        if(TIM2->CNT >= 120)
        {
            us--;
            TIM2->CNT = 0;
        }
    }

    TIM_Cmd (TIM2, DISABLE);
}

void delayTimerInit(void) 
{
    TIM_TimeBaseInitTypeDef TIM_InitStructure = { 0 };

    RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseStructInit(&TIM_InitStructure);
    TIM_InitStructure.TIM_Period = 1700 - 1;
    TIM_InitStructure.TIM_Prescaler = 1;
    //TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit (TIM2, &TIM_InitStructure);

    //TIM_Cmd (TIM2, ENABLE);
}