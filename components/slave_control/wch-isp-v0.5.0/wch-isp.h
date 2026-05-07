#ifndef WCH_ISP_UNI_H
#define WCH_ISP_UNI_H

#include <stddef.h>
#include "../dev_iface.h"

void set_function(const device_conf* dev_conf);
void cmd_write_flash_bin(uint8_t* bin, size_t size);
void cmd_write_flash();
void cmd_verify_flash_bin(uint8_t* bin, size_t size);
void cmd_verify_flash();
void cmd_config_show();
void cmd_config_set();
void cmd_remove_wp();
void cmd_erase_all();

#endif //WCH_ISP_UNI_H