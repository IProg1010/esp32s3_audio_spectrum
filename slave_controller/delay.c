#include <delay.h>
#include <stdint.h>

static uint32_t count = 0;

void TIM3_IRQHandler(void) {
    __asm volatile ("call TIM3_IRQHandler_real; mret");
}


__attribute__((used)) void TIM3_IRQHandler_real(void) 
{
    //if(TIM5-> == SET )
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
    TIM3->CNT = 0;
    TIM_Cmd (TIM3, ENABLE);

    while(us > 0)
    {
        if(TIM3->CNT >= 120)
        {
            us--;
            TIM3->CNT = 0;
        }
    }

    TIM_Cmd (TIM3, DISABLE);
}

void delayTimerInit(void) 
{
    TIM_TimeBaseInitTypeDef TIM_InitStructure = { 0 };

    RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseStructInit(&TIM_InitStructure);
    TIM_InitStructure.TIM_Period = 1700 - 1;
    TIM_InitStructure.TIM_Prescaler = 1;
    TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit (TIM3, &TIM_InitStructure);

    //TIM_Cmd (TIM2, ENABLE);
}