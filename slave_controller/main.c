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
#include "timer.h"


#define btn_enc2    1
#define btn_sb1     2
#define btn_sb2     3
#define btn_sb3     4
#define btn_sb4     5
#define btn_enc1    6

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

#define RxSize 24
#define TxSize 24
uint8_t RxBuffer1[RxSize] = { 0 };
uint8_t RxBuffer2[RxSize] = { 0 };
uint8_t TxBuffer1[TxSize] = { 0 };
uint8_t TxBuffer2[TxSize] = { 0 };
uint8_t buff_type = 0;

DMA_InitTypeDef DMA_InitStructure = {0};


void blinkLed(void);
void initPeriph(void);
void initDIO(void);
void initUart(void);
void initUSART1DMA(uint8_t* buffRX, uint16_t buffRX_size, uint8_t* buffTX, uint16_t buffTX_size);
void writeUSARTuseDMA(uint8_t* buff, uint8_t size);
void handleUart();
void handleButton();
void handleEncoder();

int main(void)
{
    initPeriph();
    while(1)
    {
        handleButton();
        handleEncoder();
        handleUart();
        
        blinkLed();
        //delay_ms(500);
        //printf("c\r\n");
        /*blinkLed();
        delay_ms(500);
        blinkLed();
        delay_ms(500);
        blinkLed();
        delay_ms(500);
        blinkLed();
        delay_ms(500);
        blinkLed();
        delay_ms(500);*/
    }
}

void handleButton()
{
    static enum PinState btn_enc2_st = stOff;
    static enum PinState btn_sb1_st = stOff;
    static enum PinState btn_sb2_st = stOff;
    static enum PinState btn_sb3_st = stOff;
    static enum PinState btn_sb4_st = stOff;
    static enum PinState btn_enc1_st = stOff;

    static enum PinState btn_enc2_st_last = stOff;
    static enum PinState btn_sb1_st_last = stOff;
    static enum PinState btn_sb2_st_last = stOff;
    static enum PinState btn_sb3_st_last = stOff;
    static enum PinState btn_sb4_st_last = stOff;
    static enum PinState btn_enc1_st_last = stOff;

    btn_enc2_st = getInputCurrentState(btn_enc2);
    btn_sb1_st = getInputCurrentState(btn_sb1);
    btn_sb2_st = getInputCurrentState(btn_sb2);
    btn_sb3_st = getInputCurrentState(btn_sb3);
    btn_sb4_st = getInputCurrentState(btn_sb4);
    btn_enc1_st = getInputCurrentState(btn_enc1);


    if(btn_enc2_st_last != btn_enc2_st)
    {
        if(btn_enc2_st == stOff)
        {
            printf("enc2 p\r\n");
        }
        else
        {
            printf("enc2 r\r\n");
        }
    }
    if(btn_sb1_st_last != btn_sb1_st)
    {
        printf("push btn sb1\r\n");
    }
    if(btn_sb2_st_last != btn_sb2_st)
    {
        printf("push btn sb2\r\n");
    }
    if(btn_sb3_st_last != btn_sb3_st)
    {
        printf("push btn sb3\r\n");
    }
    if(btn_sb4_st_last != btn_sb4_st)
    {
        printf("push btn sb4\r\n");
    }
    if(btn_enc1_st_last != btn_enc1_st)
    {
        if(btn_enc1_st == stOff)
        {
            printf("enc1 p\r\n");
        }
        else
        {
            printf("enc1 r\r\n");
        }
    }

    btn_enc2_st_last = btn_enc2_st;
    btn_sb1_st_last = btn_sb1_st;
    btn_sb2_st_last = btn_sb2_st;
    btn_sb3_st_last = btn_sb3_st;
    btn_sb4_st_last = btn_sb4_st;
    btn_enc1_st_last = btn_enc1_st;

}

void handleEncoder()
{

    uint16_t val_code = getEncoderCnt();
    int16_t val_out;
    if(val_code != 0)
    {
        int znak = 0;

        if(getEncoderDirection() == dirRight)
        {
            znak = 1;
        }
        if(getEncoderDirection() == dirLeft)
        {
            znak = -1;
        }
        val_out = val_code*znak;
        printf("enc1 val = %d \r\n", val_out);
    }

}
void handleUart()
{
    if(DMA_GetFlagStatus(DMA1_FLAG_TC5) == SET)
    {
        DMA_ClearFlag(DMA1_FLAG_TC5);

        /*if(buff_type == 0)
        {
            deinitUSART2DMA(RxBuffer2);
            currentBuff = RxBuffer1;   
            buff_type = 1; 
        }
        else
        {
            deinitUSART2DMA(RxBuffer1);
            currentBuff = RxBuffer2;
            buff_type = 0;
        }
        
        /*b1 = * ((float*) (&currentBuff[0]));
        k1 = * ((float*) (&currentBuff[4]));
        k_out1 = * ((float*) (&currentBuff[8]));    
        b2 = * ((float*) (&currentBuff[12]));
        k2 = * ((float*) (&currentBuff[16]));
        k_out2 = * ((float*) (&currentBuff[20]));*/    
        //printf("b1=%0.3f,k1=%0.3f,k_out1=%0.3f,   b2=%0.3f,k2=%0.3f,k_out2=%0.3f\r\n", b1, k1, k_out1, b2, k2, k_out2);
    
        
    }
    
    TxBuffer1[0] = 0x01;
    TxBuffer1[1] = 0x0A;
    TxBuffer1[2] = 0x0B;
    TxBuffer1[3] = 0xC1;
    TxBuffer1[4] = 0x11;
    //writeUSARTuseDMA(TxBuffer1, TxSize);
}
void initPeriph()
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    USART_Printf_Init(115200);
    
    Delay_Init();
    delayInit();
    timerGlobInit();
    

    initEncoder();
    initDIO();
    //initUSART1DMA(RxBuffer1, RxSize, TxBuffer1, TxSize);
    initUart();

    setUpdateFunctionAndPeriod(updateForTimerMs, 10);
    setUpdateFunctionAndPeriod(updateEncoder, 200);
    GPIO_Toggle_INIT();
    //initEncoder();
    
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

void initDIO()
{
/*     setIOpin(GPIO_Pin_11, pB, zummer, tOutput, InUnknow, oPP);
    setIOpin(GPIO_Pin_12, pB, start, tOutput, InUnknow, oPP);
    setIOpin(GPIO_Pin_13, pB, stop, tOutput, InUnknow, oPP);
    setIOpin(GPIO_Pin_14, pB, work_led, tOutput, InUnknow, oPP);
    setIOpin(GPIO_Pin_10, pB, error_led, tOutput, InUnknow, oPP);
    setIOpin(GPIO_Pin_2, pB, error_led, tOutput, InUnknow, oPP); */


    setIOpin(GPIO_Pin_10, pB, btn_enc2, tInput, iPU, OutUnknow);
    setIOpin(GPIO_Pin_11, pB, btn_sb1, tInput, iPU, OutUnknow);
    setIOpin(GPIO_Pin_12, pB, btn_sb2, tInput, iPU, OutUnknow);
    setIOpin(GPIO_Pin_13, pB, btn_sb3, tInput, iPU, OutUnknow);
    setIOpin(GPIO_Pin_14, pB, btn_sb4, tInput, iPU, OutUnknow);
    setIOpin(GPIO_Pin_2, pA, btn_enc1, tInput, iPU, OutUnknow);


    //setOutputStateQuickly(start, stOn);
    //setOutputStateQuickly(stop, stOn);

    setInputFilter(btn_enc2, fOn, 20);
    setInputFilter(btn_sb1, fOn, 20);
    setInputFilter(btn_sb2, fOn, 20);
    setInputFilter(btn_sb3, fOn, 20);
    setInputFilter(btn_sb4, fOn, 20);
    setInputFilter(btn_enc1, fOn, 20);

    //blinkOutputState(work_led, 100);
}


void initUart(void)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    USART_InitTypeDef USART_InitStructure = {0};

    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE );
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    /* USART1 TX-->PA.9*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; /* Configure TX Pin */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART1 RX-->PA.10*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; /* Configure RX Pin */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    USART_InitStructure.USART_BaudRate = 115200; //115200
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART1, &USART_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

void writeUSARTuseDMA(uint8_t* buff, uint8_t size)
{
    USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);
    DMA_InitTypeDef DMA_InitStructure = {0};
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA1_Channel4);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DATAR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)buff;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = size;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);


    //DMA_ClearFlag(DMA1_FLAG_TC6);
    
    DMA_Cmd(DMA1_Channel4, ENABLE);
    //DMA_Cmd(DMA1_Channel6, DISABLE);
    
    //USART_DMACmd(USART1, USART_DMAReq_Rx, DISABLE);
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
}


void initUSART1DMA(uint8_t* buffRX, uint16_t buffRX_size, uint8_t* buffTX, uint16_t buffTX_size)
{
    //uint8_t r = USART1->DATAR;
    //USART_ClearFlag(USART1, USART_FLAG_RXNE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DATAR); /* USART1->DATAR:0x40004404 */
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)buffRX;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = buffRX_size;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);
    
    DMA_DeInit(DMA1_Channel4);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DATAR); /* USART1->DATAR:0x40004404 */
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)buffTX_size;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = TxSize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);

    //DMA_ClearFlag(DMA1_FLAG_TC6);
    
    DMA_Cmd(DMA1_Channel5, ENABLE);
    DMA_Cmd(DMA1_Channel4, ENABLE);
    //DMA_Cmd(DMA1_Channel5, DISABLE);
    //DMA_Cmd(DMA1_Channel4, DISABLE);
    

    //USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);
    //USART_DMACmd(USART1, USART_DMAReq_Rx, DISABLE);

    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
}

void deinitUSART1DMARx(uint8_t* buff)
{
    //USART_ClearFlag(USART1, USART_FLAG_RXNE);
    DMA_Cmd(DMA1_Channel5, DISABLE);
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)buff;
    DMA_InitStructure.DMA_BufferSize = RxSize;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);
    
    DMA_Cmd(DMA1_Channel5, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
}