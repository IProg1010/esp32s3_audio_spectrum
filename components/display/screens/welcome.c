#include "lvgl.h"

lv_obj_t * welcome_screen; 

void welcome_screen_init(void)
{
    welcome_screen = lv_obj_create(NULL); // Создаем пустой экран
    static lv_style_t style_text;
    lv_style_init(&style_text);

    // Создаем кнопку
    //lv_obj_t * btn = lv_btn_create(welcome_screen);
    //lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0);

    lv_style_set_text_font(&style_text, &lv_font_montserrat_22);
    // Добавляем текст
    lv_obj_t * label = lv_label_create(welcome_screen);
    
    lv_obj_add_style(label, &style_text, 0);    
    lv_label_set_text(label, "Hello Batyrshin Ilnaz");
}    