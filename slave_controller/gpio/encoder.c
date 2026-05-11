#include "encoder.h"
#include "debug.h"
#include "ch32v20x_tim.h"
#include "ch32v20x_gpio.h"
#include "math.h"


uint16_t encoder_cnt = 0;
uint16_t encoder_last = 0;

uint16_t enc_value = 0;
uint16_t enc_speed = 0;

uint16_t enc_temp_cnt = 0;

enum EncoderDir enc_dir = dirNot;

void TIM2_Encoder_Init();

void initEncoder()
{
    encoder_cnt = 0;
    TIM2_Encoder_Init();
}

uint16_t getEncoderCnt()
{
    uint16_t retVal = enc_value;
    enc_value = 0;
    return retVal;
}

uint16_t checkEncoderCnt()
{
    return TIM2->ATRLR;
}

uint8_t getSpeed()
{
    return enc_speed;
}

enum EncoderDir getEncoderDirection()
{
    return enc_dir;
}

void updateEncoder()
{
    encoder_cnt = TIM2->CNT;
    if(encoder_cnt > encoder_last)
    {
        if(encoder_cnt > 1500 && encoder_last < 100)
        {   
            enc_temp_cnt += 1600 - encoder_cnt + encoder_last;
        }
        else
        {
            enc_temp_cnt += encoder_cnt - encoder_last;
        }
            
        enc_dir = dirLeft;
    }
    else if(encoder_cnt < encoder_last)
    {
        if(encoder_last > 1500 && encoder_cnt < 100)
        {   
            enc_temp_cnt += 1600 - encoder_last + encoder_cnt;
        }
        else
        {
            enc_temp_cnt += encoder_last - encoder_cnt;
        }

        enc_dir = dirRight;
    }
    /*else
    {
        enc_dir = dirNot; 
    }*/

    if(enc_temp_cnt/4 > 0)
    {
        enc_value += enc_temp_cnt/4;
        enc_temp_cnt = enc_temp_cnt%4;
    }
    enc_speed = encoder_cnt - encoder_last;

    encoder_last = encoder_cnt; 
}

void TIM2_Encoder_Init()
{
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
    TIM_TimeBaseStructure.TIM_Period = 1600;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising , TIM_ICPolarity_Rising);

    TIM_ICStructInit(&TIM_ICInitStructure);
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 16;

    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    TIM_SetCounter(TIM2, 800);
    TIM_Cmd(TIM2, ENABLE);
    //TIM2->CNT = 1600;
}

void TIM4_Encoder_Init()
{
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
    TIM_TimeBaseStructure.TIM_Period = 1600;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising , TIM_ICPolarity_Rising);

    TIM_ICStructInit(&TIM_ICInitStructure);
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 16;

    TIM_ICInit(TIM4, &TIM_ICInitStructure);

    TIM_SetCounter(TIM4, 800);
    TIM_Cmd(TIM4, ENABLE);
    //TIM4->CNT = 1600;
}