#include "lvgl.h"
#include "input_manager.h"
#include <stdio.h>
// Глобальная переменная экрана, чтобы к ней можно было обратиться извне
lv_obj_t * main_screen;
lv_group_t * main_screen_group;

static void btn_dac_event_handle(lv_event_t * e);
static void btn_adc_event_handle(lv_event_t * e);



void main_screen_init(void) 
{
    main_screen = lv_obj_create(NULL); // Создаем пустой экран

    // Делаем экран контейнером для флексов
    lv_obj_set_layout(main_screen, LV_LAYOUT_FLEX);

    // Настраиваем: Колонка, прижатая к левому верхнему углу
    lv_obj_set_flex_flow(main_screen, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(main_screen, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    /*// Создаем кнопку
    lv_obj_t * btn = lv_btn_create(main_screen);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0);

    // Добавляем текст
    lv_obj_t * label = lv_label_create(btn);
    lv_label_set_text(label, "Start");*/

    main_screen_group = lv_group_create();
    lv_group_set_default(main_screen_group); // Все новые объекты по умолчанию попадут сюда

    input_set_enc1_group(main_screen_group);
    // Привязываем энкодер к этой группе
    // encoder_indev — это твой объект lv_indev_t, настроенный в main.cpp
    //extern lv_indev_t * encoder_indev; 
    //lv_indev_set_group(encoder_indev, main_screen_group);

    // Создаем кнопку
    lv_obj_t * btn_dac = lv_btn_create(main_screen);
    //lv_obj_align(btn_dac, LV_DIR_LEFT, 0, 0);
    
    // ПРИВЯЗЫВАЕМ СОБЫТИЕ
    lv_obj_add_event_cb(btn_dac, btn_dac_event_handle, LV_EVENT_ALL, NULL);

    lv_obj_t * label_dac = lv_label_create(btn_dac);
    lv_label_set_text(label_dac, "DAC");


    // Создаем кнопку
    lv_obj_t * btn_adc = lv_btn_create(main_screen);
    //lv_obj_align(btn_adc, LV_DIR_LEFT, 0, 0);
    
    // ПРИВЯЗЫВАЕМ СОБЫТИЕ
    lv_obj_add_event_cb(btn_adc, btn_adc_event_handle, LV_EVENT_ALL, NULL);

    lv_obj_t * label_adc = lv_label_create(btn_adc);
    lv_label_set_text(label_adc, "ADC");
}

static void btn_dac_event_handle(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);

    if(code == LV_EVENT_CLICKED) {
        // Логика нажатия: например, меняем текст или переменную
        printf("Button1 Clicked!\n");
    }
}

static void btn_adc_event_handle(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);

    if(code == LV_EVENT_CLICKED) {
        // Логика нажатия: например, меняем текст или переменную
        printf("Button2 Clicked!\n");
    }
}
