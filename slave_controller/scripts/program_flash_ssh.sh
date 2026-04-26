#!/bin/bash

conig_openocd="/home/tpp/work/ch32/ch32tools/OpenOCD/bin/wch-riscv.cfg"
flash_file="../flash.bin"

openocd -f $conig_openocd -c init -c halt -c "flash write_image erase $flash_file 0x00000000 verify reset exit" -c "reset" -c shutdown
