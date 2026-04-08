/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : ASUPROM
* Version            : V1.0.0
* Date               : 2024/10/10
* Description        : Main program body.
*/

#include "debug.h"
#include "adc_control.h"
#include "delay.h"
#include "lcd.h"
#include "servo.h"
#include "motor.h"

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

    printf("Start servo program\r\n");
    
    float var;
    int i = 0;
    int cycle =0;
    while(1)
    {
        //printf("Get id = %3.2f", var);
        //Delay_Ms(250);
        delay_ms(100);
        if(i == 0)
        {
            //Ethernet_LED_DATASET(0);
            //Ethernet_LED_LINKSET(0);
            GPIO_WriteBit(GPIOB, GPIO_Pin_3, Bit_RESET);
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
            GPIO_WriteBit(GPIOB, GPIO_Pin_3, Bit_SET);
            GPIO_WriteBit(GPIOB, GPIO_Pin_4, Bit_RESET);
            //printf("Led OFF:%.3f\r\n", adc_var);
            //printf("Led OFF\r\n");
            //sendStr("Led OFF", 1, 1);

            i--;
        }

        lcdPrintf(3, 3, "cycle = %d", cycle);
        cycle++;
        /*for(int i = 0; i < 180; i += 20)
        {
            servoSetAngle(i);
            Delay_Ms(2000);
        }*/

            /*motorSetDirect(dForw);
            delay_Ms(5000);

            servoSetAngle(0);
            Delay_Ms(100);

            servoSetAngle(90);
            Delay_Ms(100);

            servoSetAngle(179);
            Delay_Ms(100);
            motorSetDirect(dBack);
            Delay_Ms(5000);*/
    }
}

#define BUFF_SIZE_SYMB 15
char buff_symb[BUFF_SIZE_SYMB] = { 0 };
uint8_t buff_symb_i = 0;

void add_char(char ch)
{
    if(buff_symb_i+1 < BUFF_SIZE_SYMB)
    {
        buff_symb[buff_symb_i] = ch;
        buff_symb_i++;
    }
    else
    {
        for(int i = 1; i < BUFF_SIZE_SYMB; i++)
        {
            buff_symb[i-1] = buff_symb[i];
        }
        buff_symb[BUFF_SIZE_SYMB-1] = ch;
    }

    lcdPrintf(1, 1, "SYM: %s", buff_symb);
}

void printLCD(int type, int32_t data)
{
    //if(data != 0)
    {
        if(type == 0)
        {
            lcdPrintf(1, 1, "%d = %d", type, data);
        }
        if(type == 1)
        {
            lcdPrintf(1, 10, "%d = %d", type, data);
        }
        if(type == 2)
        {
            lcdPrintf(2, 1, "%d = %d", type, data);
        }
        if(type == 3)
        {
            lcdPrintf(2, 10, "%d = %d", type, data);
        }
        if(type == 4)
        {
            lcdPrintf(3, 1, "%d = %d", type, data);
        }
        if(type == 5)
        {
            lcdPrintf(3, 10, "%d = %d", type, data);
        }
        if(type == 6)
        {
            lcdPrintf(4, 1, "%d = %d", type, data);
        }
        if(type == 7)
        {
            lcdPrintf(4, 10, "%d = %d", type, data);
        }
    }
    /*else
    {

    }*/
}


void initPeriph()
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Delay_Init();
    delayInit();
    setPrintFunction(printLCD);
    setAddCharFunction(add_char);
    USART_Printf_Init(115200);

    lcdAllInit();

    char buff_text[] = {"Hello world!"};
    lcdPrintf(2, 3, buff_text);

    GPIO_Toggle_INIT();

    initServo();
    initMotor();

    adcInit();
    //ADC_Function_Init();

    // setup timer for tiny-macro-os and lwip sys_now 
    
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
