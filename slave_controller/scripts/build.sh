#!/bin/bash

script_dir=${PWD}

cd ..

mkdir build
mkdir output_files
cd output_files

output_dir=${PWD}
echo -en "Output directory: "$output_dir"\n"

cd ..

cd build

cmake -S ../ -B ./

make

git_rev=$(git log --pretty=format:'%h' -n 1)

cp ./slave_controller_$git_rev.bin $output_dir/flash_slave.bin
cp ./slave_controller.elf $output_dir/flash_slave.elf
