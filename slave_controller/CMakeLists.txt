cmake_minimum_required(VERSION 3.20)

# Optional: print out extra messages to see what is going on. Comment it to have less verbose messages
set(CMAKE_VERBOSE_MAKEFILE ON)

# Path to toolchain file. This one has to be before 'project()' below
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/toolchain.cmake)
set(GIT_HEAD_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.git/HEAD")

project(dtmf C CXX ASM)

#set(PATH_TO_SDK ${CMAKE_SOURCE_DIR}/../../cmsis CACHE PATH "Path to foled with sdk for chip of stm32f1X")

set(EXECUTABLE ${PROJECT_NAME}.elf)
set(LINKER_FILE ${CMAKE_SOURCE_DIR}/memory_link.ld)

set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_ASM_FLAGS "-x assembler-with-cpp")
#set_property(SOURCE startup_stm32f103xb.s APPEND PROPERTY COMPILE_OPTIONS "-x" "assembler-with-cpp")

message("PAth to ${PATH_TO_SDK}")
message("Cmake source dir" ${CMAKE_SOURCE_DIR})

set(ch32tools "/home/batyrshin/work/ch32/ch32tools/ch32v20x/C++/Use\ MRS\ Create\ C++\ project-example/CH32V203C8T6++")
set(ch32v203core "Core")
set(ch32v203debug "Debug")
set(ch32v203peripheral "Peripheral")
set(ch32v203startup "Startup")
set(ch32v203user "User")

# List of source files
set(SOURC_FILES 
	${CMAKE_SOURCE_DIR}/main.cpp
	${CMAKE_SOURCE_DIR}/delay.c
	${CMAKE_SOURCE_DIR}/periph/servo.c
	${CMAKE_SOURCE_DIR}/periph/motor.c
	${CMAKE_SOURCE_DIR}/periph/gpio.c
	${CMAKE_SOURCE_DIR}/periph/esp01.c
	${CMAKE_SOURCE_DIR}/lcd/lcd.c
	${CMAKE_SOURCE_DIR}/adc/adc_control.c
	${CMAKE_SOURCE_DIR}/adc/goertzelfilter.c

	${ch32tools}/${ch32v203core}/core_riscv.c

	${ch32tools}/${ch32v203user}/system_ch32v20x.c
	${ch32tools}/${ch32v203startup}/startup_ch32v20x_D6.S
	${ch32tools}/${ch32v203user}/ch32v20x_it.c
	${ch32tools}/${ch32v203peripheral}/src/ch32v20x_gpio.c
	${ch32tools}/${ch32v203peripheral}/src/ch32v20x_usart.c
	${ch32tools}/${ch32v203peripheral}/src/ch32v20x_i2c.c
	${ch32tools}/${ch32v203peripheral}/src/ch32v20x_spi.c
	${ch32tools}/${ch32v203peripheral}/src/ch32v20x_rcc.c
	${ch32tools}/${ch32v203peripheral}/src/ch32v20x_dbgmcu.c
	${ch32tools}/${ch32v203peripheral}/src/ch32v20x_misc.c
	${ch32tools}/${ch32v203peripheral}/src/ch32v20x_adc.c
	${ch32tools}/${ch32v203peripheral}/src/ch32v20x_dma.c
	${ch32tools}/${ch32v203peripheral}/src/ch32v20x_tim.c
	#${ch32tools}/${ch32v203peripheral}/src/ch32v20x_eth.c
	#${ch32tools}/${ch32v203peripheral}/src/ch32v20x_rng.c
	${ch32tools}/${ch32v203debug}/debug.c
)

# Build the executable based on the source files
add_executable(${EXECUTABLE} ${SOURC_FILES})

# List of compiler defines, prefix with -D compiler option
target_compile_definitions(${EXECUTABLE} PRIVATE
	-D__DEBUG_MODE__
	)

# List of includ directories
target_include_directories(${EXECUTABLE} PUBLIC
	${CMAKE_SOURCE_DIR}/

	${ch32tools}/${ch32v203core}/
	${CMAKE_SOURCE_DIR}/periph/
	${CMAKE_SOURCE_DIR}/lcd/
	${CMAKE_SOURCE_DIR}/adc/

	${ch32tools}/${ch32v203user}/
	${ch32tools}/${ch32v203peripheral}/inc/
	${ch32tools}/${ch32v203debug}/
	)

include_directories(
	${CMAKE_SOURCE_DIR}/

	${ch32tools}/${ch32v203core}/
	${CMAKE_SOURCE_DIR}/periph/
	${CMAKE_SOURCE_DIR}/lcd/
	${CMAKE_SOURCE_DIR}/adc/

	${ch32tools}/${ch32v203user}/
	${ch32tools}/${ch32v203peripheral}/inc/
	${ch32tools}/${ch32v203debug}/
	)

# Compiler options
target_compile_options(${EXECUTABLE} PRIVATE
	#-mcpu=
	#-mthumb
	#-march=rv32i_zicsr
	#-S -o
	#-march=rv32imac_zicsr_zifencei
	-march=rv32imac_zicsr
	#-with-arch=rv32gc
	#-march=rv32imac_zicsr
	-g 
	-Os 
	-flto 
	-ffunction-sections 
	-fdata-sections 
	-fmessage-length=0 
	#-msmall-data-limit=8
	#-march=rv32imafc 
	#-mabi=ilp32f
	-static-libgcc 
	-DCH32V20x_D6
	#-nostdlib
	-Wall
	-O1
	#-specs=nosys.specs
	)

# Linker options
target_link_options(${EXECUTABLE} PUBLIC
	-T${LINKER_FILE}
	#-mcpu=cortex-m4
	#-mthumb
	#-specs=nano.specs
	-specs=nano.specs 
	-specs=nosys.specs
	-Wl,--gc-sections
	-lgcc
	-nostartfiles
	-static-libgcc
	-lstdc++
	-lg
	-u _printf_float #for ptintf float
	-lc
	)

add_custom_command(TARGET ${EXECUTABLE}
	POST_BUILD
	COMMAND ${CMAKE_SIZE_UTIL} ${EXECUTABLE})

	execute_process(COMMAND git log --pretty=format:'%h' -n 1
	OUTPUT_VARIABLE GIT_REV
	ERROR_QUIET)

if ("${GIT_REV}" STREQUAL "")
	set(GIT_REV "N/A")
	set(GIT_DIFF "")
	set(GIT_TAG "N/A")
	set(GIT_BRANCH "N/A")
else()
	execute_process(
			COMMAND bash -c "git diff --quiet --exit-code || echo +"
			OUTPUT_VARIABLE GIT_DIFF)
	execute_process(
			COMMAND git describe --exact-match --tags
			OUTPUT_VARIABLE GIT_TAG ERROR_QUIET)
	execute_process(
		COMMAND git rev-parse --abbrev-ref HEAD
		OUTPUT_VARIABLE GIT_BRANCH)

	string(STRIP "${GIT_REV}" GIT_REV)
	string(SUBSTRING "${GIT_REV}" 1 7 GIT_REV)
	string(STRIP "${GIT_DIFF}" GIT_DIFF)
	string(STRIP "${GIT_TAG}" GIT_TAG)
	string(STRIP "${GIT_BRANCH}" GIT_BRANCH)
endif()

# Optional: Create hex, bin and S-Record files after the build
add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O srec --srec-len=64 ${EXECUTABLE} ${PROJECT_NAME}.s19
        COMMAND ${CMAKE_OBJCOPY} -O ihex ${EXECUTABLE} ${PROJECT_NAME}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary ${EXECUTABLE} ${PROJECT_NAME}_${GIT_REV}.bin)
