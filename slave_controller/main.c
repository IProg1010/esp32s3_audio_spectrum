/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : ASUPROM
* Version            : V1.0.0
* Date               : 2024/10/10
* Description        : Main program body.
*/

#include "debug.h"
#include "delay.h"
#include "di_do_control.h"
#include "encoder.h"

/*
gpio 
-PA1 - analog input from mircophone

LED:
- PB3, PB4 - led_1, led_2

DEBUG UART:
- PA9, PA10 - debug port (tx, rx)

ESP32:
- PB10, PB11 - uart for esp32 (tx, rx)
- PA12 - enable pin

lcd port:
- PB0, PB1, PB12, PB13 - D1, D2, D3, D4
- PB14, PB15 - RS, EN
 
*/
/*********************************************************************
 * @fn      GPIO_Toggle_INIT
 *
 * @brief   Initializes GPIOA.0
 *
 * @return  none
 */
void GPIO_Toggle_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void initPeriph(void);

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */

int main(void)
{
    initPeriph();
    while(1)
    {
    }
}



void initPeriph()
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Delay_Init();
    delayInit();
    USART_Printf_Init(115200);

    
    GPIO_Toggle_INIT();

    
    /*SysTick->SR &= ~(1 << 0);//clear State flag
    SysTick->CMP = 500000;
    SysTick->CNT = 0;
    SysTick->CTLR = 0xF;

    NVIC_SetPriority(SysTicK_IRQn, 10);
    NVIC_EnableIRQ(SysTicK_IRQn);*/
}

void blinkLed()
{
    static uint32_t currTime = 0;
    static int i = 0;
    //if(getTime() - currTime > 5)
    {
       // printf("led blink\r\n");
        if(i == 0)
        {
            //Ethernet_LED_DATASET(0);
            //Ethernet_LED_LINKSET(0);
            GPIO_WriteBit(GPIOB, GPIO_Pin_4, Bit_SET);
           //adc_var = 12.368;
            //printf("Led ON:%.3f\r\n", adc_var);
            //printf("Led ON:\r\n");
            //sendStr("Led ON", 1, 1);
            i++;
        }
        else
        {
            //adc_var = 101.290;
            //Ethernet_LED_DATASET(1);
            //Ethernet_LED_LINKSET(1);
            GPIO_WriteBit(GPIOB, GPIO_Pin_4, Bit_RESET);
            //printf("Led OFF:%.3f\r\n", adc_var);
            //printf("Led OFF\r\n");
            //sendStr("Led OFF", 1, 1);
            i--;
        }
    }
}
