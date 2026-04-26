#!/bin/bash

conig_openocd="/home/batyrshin/work/ch32/ch32tools/OpenOCD/bin/wch-riscv.cfg"
flash_file="../output_files/flash.bin"

openocd -f $conig_openocd -c init -c halt -c "flash write_image erase $flash_file 0x00000000 verify reset exit" -c "reset" -c shutdown
