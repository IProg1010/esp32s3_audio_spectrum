#include <timer.h>
#include <debug.h>

/*timer for display update*/

struct UpdateFuncStruct
{
    uint16_t ticks_ms;
    void (*funct_ptr) ();
    uint16_t current_tick;
};

uint8_t upd_funct_cnt = 0;
struct UpdateFuncStruct upd_funct_arr[30];
struct UpdateFuncStruct* upd_funct_arr_last;

void (*funct_arr[30]) ()  = { NULL };
uint8_t funct_cnt = 0;

uint16_t last_tick = 0;
uint16_t ticks = 1000;
uint8_t update_flag = 0;

#define TIMER_COUNT 32
typedef struct
{
    uint32_t tick;
    int num;
} timer_tick;

volatile timer_tick timer_tick_arr[TIMER_COUNT+1];
volatile uint8_t timer_tick_num[TIMER_COUNT+1];
volatile uint8_t timer_tick_curr;
/*void TIM1_IRQHandler(void) {
    __asm volatile ("call TIM1_IRQHandler_real; mret");
}

__attribute__((used)) void TIM1_IRQHandler_real(void) 
{
    if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        update_flag = 1;
    
        //printf("Update flag %d \r\n", TIM1->CNT);
        for(int i = 0; i < upd_funct_cnt; i++)
        {
            upd_funct_arr[i].current_tick++;
        }
        printf("this is glob timer");
    }
	
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
}*/
void TIM1_UP_IRQHandler(void) {
    __asm volatile ("call TIM1_UP_IRQHandler_real; mret");
}

__attribute__((used)) void TIM1_UP_IRQHandler_real(void) 
{
    if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        update_flag = 1;
    
        //printf("Update flag %d \r\n", TIM1->CNT);
        for(int i = 0; i < upd_funct_cnt; i++)
        {
            upd_funct_arr[i].current_tick++;
            updateGlobTimer2();
        }
        
        for(int i = 0; i < timer_tick_curr; i++)
        {
            timer_tick_arr[i].tick++;
        }
        //printf("this is glob timer");
    }
}


void timerInit(void) 
{
    TIM_TimeBaseInitTypeDef TIM_InitStructure = { 0 };

    RCC_APB2PeriphClockCmd (RCC_APB2Periph_TIM1, ENABLE);

    TIM_TimeBaseStructInit(&TIM_InitStructure);
    TIM_InitStructure.TIM_Period = 36000;//1000;// 65535;
    TIM_InitStructure.TIM_Prescaler = 3;
    TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM1, &TIM_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure={0};
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    TIM_Cmd(TIM1, ENABLE);
}

void timerGlobInit(void)
{
    timerInit();
}

uint16_t getTicks()
{   

}

void updateGlobTimer()
{
    if(TIM1->CNT > ticks)
    {
        //printf("Update flag %d \r\n", TIM1->CNT);
        for(int i = 0; i < funct_cnt; i++)
        {
            funct_arr[i]();
        }
        TIM1->CNT = 0;
    }
}

void updateGlobTimer2()
{
    for(int i = 0; i < upd_funct_cnt; i++)
    {
        if(upd_funct_arr[i].current_tick > upd_funct_arr[i].ticks_ms)
        {
            //printf("Update flag %d \r\n", TIM1->CNT);
            upd_funct_arr[i].funct_ptr();
            upd_funct_arr[i].current_tick = 0;
        }
    }
}


void setUpdateFunction(void (*update_ptr)())
{
    funct_arr[funct_cnt] = update_ptr;
    funct_cnt++;
}

void setUpdateFunctionAndPeriod(void (*update_ptr)(), uint16_t period_ms)
{
    upd_funct_arr[upd_funct_cnt].funct_ptr = update_ptr;
    upd_funct_arr[upd_funct_cnt].ticks_ms = period_ms;
    upd_funct_arr[upd_funct_cnt].current_tick = 0;
    upd_funct_cnt++;
}

void startTimerTick(int num)
{
    if(num <= TIMER_COUNT)
    {
        if(timer_tick_curr > TIMER_COUNT)
        {
            timer_tick_curr = 0;
        }

        for(int i = 0; i <= TIMER_COUNT; i++)
        {
            timer_tick_num[timer_tick_curr++] = num;
            timer_tick_arr[num].tick = 0;
        }
    }
}

uint32_t getTimerTick(int num)
{
    if(num <= TIMER_COUNT)
    {
        return timer_tick_arr[num].tick;
    }
}
/*timer for display update in home screen*/