#ifndef DEV_IFACE_H
#define DEV_IFACE_H

#include <stdint.h>

typedef struct
{
    int (*write) (uint8_t* buff, uint16_t size);
    int (*read) (uint8_t* buff, uint16_t size, int* get_cnt);
} device_conf;

#endif //DEV_IFACE_H