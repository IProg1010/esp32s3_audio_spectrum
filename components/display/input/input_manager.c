#include "input_manager.h"

static QueueHandle_t input_queue1;
static QueueHandle_t input_queue2;

static lv_indev_t* indev_enc1;
static lv_indev_t* indev_enc2;

// Общая переменная состояния (упростим: для каждого энкодера свои статики)
static int16_t e1_diff = 0; bool e1_pres = false;
static int16_t e2_diff = 0; bool e2_pres = false;

// CALLBACK для Энкодера 1
static void read_enc1(lv_indev_t * indev, lv_indev_data_t * data) 
{
    input_event_t ev;
    // Очередь и логика остаются прежними
    if(xQueueReceive(input_queue1, &ev, 0) == pdTRUE)
    {
        if(ev == ENC_UP) {
            data->enc_diff = 1;
        } 
        else if(ev == ENC_DOWN) {
            data->enc_diff = -1;
        } 
        else if(ev == ENC_PUSH) {
            e1_pres = true;
        } 
        else if(ev == ENC_RELEASE) {
            e1_pres = false;
        }

        if(uxQueueMessagesWaiting(input_queue1) > 0) 
        {
            data->continue_reading = true;
        }
    }
    else 
    {
        // Если очередь пуста, сбрасываем diff
        data->enc_diff = 0;
    }
    
    //data->enc_diff = e1_diff;
    //e1_diff = 0;
    data->state = e1_pres ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
}
// CALLBACK для Энкодера 2 (аналогично)
static void read_enc2(lv_indev_t * indev, lv_indev_data_t * data) 
{
    input_event_t ev;
    // Очередь и логика остаются прежними
    while(xQueueReceive(input_queue2, &ev, 0) == pdTRUE) {
        if(ev == ENC_UP) e2_diff++;
        else if(ev == ENC_DOWN) e2_diff--;
        else if(ev == ENC_PUSH) e2_pres = true;
        else if(ev == ENC_RELEASE) e2_pres = false;
    }
    
    data->enc_diff = e2_diff;
    e2_diff = 0;
    data->state = e2_pres ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
}

void input_manager_init()
{
    input_queue1 = xQueueCreate(32, sizeof(input_event_t));
    input_queue2 = xQueueCreate(32, sizeof(input_event_t));

    // Создаем устройство ввода для Энкодера 1
    indev_enc1 = lv_indev_create();
    lv_indev_set_type(indev_enc1, LV_INDEV_TYPE_ENCODER);
    lv_indev_set_read_cb(indev_enc1, read_enc1);

    // Создаем устройство ввода для Энкодера 2
    indev_enc2 = lv_indev_create();
    lv_indev_set_type(indev_enc2, LV_INDEV_TYPE_ENCODER);
    lv_indev_set_read_cb(indev_enc2, read_enc2);
}

QueueHandle_t get_input_queue_enc1() { return input_queue1; }
QueueHandle_t get_input_queue_enc2() { return input_queue2; }

void input_set_enc1_group(lv_group_t* g) { lv_indev_set_group(indev_enc1, g); }
void input_set_enc2_group(lv_group_t* g) { lv_indev_set_group(indev_enc2, g); }