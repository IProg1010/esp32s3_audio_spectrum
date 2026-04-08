#include "encoder.h"
#include "debug.h"
#include "ch32v30x_tim.h"
#include "ch32v30x_gpio.h"
#include "math.h"


uint16_t encoder_cnt = 0;
uint16_t encoder_last = 0;

uint16_t enc_value = 0;
uint16_t enc_speed = 0;

uint16_t enc_temp_cnt = 0;

enum EncoderDir enc_dir = dirNot;

void TIM8_Encoder_Init();

void initEncoder()
{
    encoder_cnt = 0;
    TIM8_Encoder_Init();
}

uint16_t getEncoderCnt()
{
    uint16_t retVal = enc_value;
    enc_value = 0;
    return retVal;
}

uint16_t checkEncoderCnt()
{
    return TIM8->ATRLR;
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
    encoder_cnt = TIM8->CNT;
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
/*
#define ECR_PORTPINCONFIG_MASK    ((uint16_t)0xFF80)
#define LSB_MASK                  ((uint16_t)0xFFFF)
#define DBGAFR_POSITION_MASK      ((uint32_t)0x000F0000)
#define DBGAFR_SWJCFG_MASK        ((uint32_t)0xF0FFFFFF)
#define DBGAFR_LOCATION_MASK      ((uint32_t)0x00200000)
#define DBGAFR_NUMBITS_MASK       ((uint32_t)0x00100000)
void GPIO_PinRemapConfigTest(uint32_t GPIO_Remap, FunctionalState NewState)
{
    uint32_t tmp = 0x00, tmp1 = 0x00, tmpreg = 0x00, tmpmask = 0x00;

    if((GPIO_Remap & 0x80000000) == 0x80000000)
    {
        tmpreg = AFIO->PCFR2;
    }
    else
    {
        tmpreg = AFIO->PCFR1;
    }

    tmpmask = (GPIO_Remap & DBGAFR_POSITION_MASK) >> 0x10;
    tmp = GPIO_Remap & LSB_MASK;

    /// Clear bit 
    if((GPIO_Remap & 0x80000000) == 0x80000000)
    {                                                                                                                   // PCFR2 
        if((GPIO_Remap & (DBGAFR_LOCATION_MASK | DBGAFR_NUMBITS_MASK)) == (DBGAFR_LOCATION_MASK | DBGAFR_NUMBITS_MASK)) // [31:16] 2bit 
        {
            tmp1 = ((uint32_t)0x03) << (tmpmask + 0x10);
            tmpreg &= ~tmp1;
        }
        else if((GPIO_Remap & DBGAFR_NUMBITS_MASK) == DBGAFR_NUMBITS_MASK) // [15:0] 2bit 
        {
            tmp1 = ((uint32_t)0x03) << tmpmask;
            tmpreg &= ~tmp1;
        }
        else // [31:0] 1bit 
        {
            tmpreg &= ~(tmp << ((GPIO_Remap >> 0x15) * 0x10));
        }
    }
    else
    {                                                                                                                   // PCFR1 
        if((GPIO_Remap & (DBGAFR_LOCATION_MASK | DBGAFR_NUMBITS_MASK)) == (DBGAFR_LOCATION_MASK | DBGAFR_NUMBITS_MASK)) // [26:24] 3bit SWD_JTAG 
        {
            tmpreg &= DBGAFR_SWJCFG_MASK;
            AFIO->PCFR1 &= DBGAFR_SWJCFG_MASK;
        }
        else if((GPIO_Remap & DBGAFR_NUMBITS_MASK) == DBGAFR_NUMBITS_MASK) // [15:0] 2bit 
        {
            tmp1 = ((uint32_t)0x03) << tmpmask;
            tmpreg &= ~tmp1;
            tmpreg |= ~DBGAFR_SWJCFG_MASK;
        }
        else // [31:0] 1bit 
        {
            tmpreg &= ~(tmp << ((GPIO_Remap >> 0x15) * 0x10));
            tmpreg |= ~DBGAFR_SWJCFG_MASK;
        }
    }

    //Set bit 
    if(NewState != DISABLE)
    {
        tmpreg |= (tmp << ((GPIO_Remap >> 0x15) * 0x10));
    }

    if((GPIO_Remap & 0x80000000) == 0x80000000)
    {
        
        printf("tmpreg = %x\r\n", tmpreg);
        //AFIO->PCFR2 = tmpreg;
    }
    else
    {
        printf("tmpreg = %x\r\n", tmpreg);
        //AFIO->PCFR1 = tmpreg;
    }
}*/

void TIM8_Encoder_Init()
{
    //#undef GPIO_Remap_TIM8
    //#define GPIO_Remap_TIM8             ((uint32_t)0x80130004)  /* TIM8 Alternate Function mapping */
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinRemapConfig(GPIO_Remap_TIM8, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_TIM8, DISABLE);
    GPIO_PinRemapConfig(GPIO_Remap_TIM8, ENABLE);
    //GPIO_PinRemapConfigTest(GPIO_Remap_TIM8, ENABLE);
    //GPIO_PinRemapConfigTest(GPIO_FullRemap_TIM9, ENABLE);
    //GPIO_PinRemapConfig(GPIO_FullRemap_TIM9, ENABLE);
    //AFIO->PCFR1 |= 0x00000800; 

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
    TIM_TimeBaseStructure.TIM_Period = 1600;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

    TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising , TIM_ICPolarity_Rising);

    TIM_ICStructInit(&TIM_ICInitStructure);
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 16;

    TIM_ICInit(TIM8, &TIM_ICInitStructure);

    /*NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;

    NVIC_Init(&NVIC_InitStructure);*/

    /*TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);*/
    //GPIO_PinRemapConfig(GPIO_Remap_TIM8, ENABLE);
    //AFIO->PCFR2 |= 0x4; 
    TIM_SetCounter(TIM8, 800);
    TIM_Cmd(TIM8, ENABLE);
    //TIM8->CNT = 1600;
}