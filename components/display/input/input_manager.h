#ifndef INPUT_MANAGER_H
#define INPUT_MANAGER_H

#include "lvgl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Типы событий для очереди
typedef enum {
    ENC_UP, ENC_DOWN, ENC_PUSH, ENC_RELEASE
} input_event_t;

// Инициализация
void input_manager_init();

// Получить дескриптор очереди (для функции парсинга UART)
QueueHandle_t get_input_queue_enc1();
QueueHandle_t get_input_queue_enc2();

// Привязка групп к конкретным энкодерам
void input_set_enc1_group(lv_group_t* group);
void input_set_enc2_group(lv_group_t* group);

#endif