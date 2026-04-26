#!/bin/bash

. ./build.sh

output_dir="output_files"
file_name="flash.bin"

scp -P 23587  ../$output_dir/$file_name tpp@192.168.1.27:/home/tpp/work/ch32/projects/ch32V307_pwr400/file_name

ssh -p 23587 tpp@192.168.1.27 'cd ~/work/ch32/projects/ch32V307_pwr400/scripts; bash -s < ./program_flash_ssh.sh'
