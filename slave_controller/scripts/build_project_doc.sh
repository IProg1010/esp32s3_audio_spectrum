#!/bin/bash

#Памятка, Таблица цветов и фонов

#Цвет           код       код фона



#black    30  40    \033[30m  \033[40m

#red      31  41    \033[31m  \033[41m

#green    32  42    \033[32m  \033[42m

#yellow    33  43    \033[33m  \033[43m

#blue    34  44    \033[34m  \033[44m

#magenta    35  45    \033[35m  \033[45m

#cyan    36  46    \033[36m  \033[46m

#white    37  47    \033[37m  \033[47m



# Дополнительные свойства для текта:

BOLD='\033[1m'       #  ${BOLD}      # жирный шрифт (интенсивный цвет)

DBOLD='\033[2m'      #  ${DBOLD}    # полу яркий цвет (тёмно-серый, независимо от цвета)

NBOLD='\033[22m'      #  ${NBOLD}    # установить нормальную интенсивность

UNDERLINE='\033[4m'     #  ${UNDERLINE}  # подчеркивание

NUNDERLINE='\033[4m'     #  ${NUNDERLINE}  # отменить подчеркивание

BLINK='\033[5m'       #  ${BLINK}    # мигающий

NBLINK='\033[5m'       #  ${NBLINK}    # отменить мигание

INVERSE='\033[7m'     #  ${INVERSE}    # реверсия (знаки приобретают цвет фона, а фон -- цвет знаков)

NINVERSE='\033[7m'     #  ${NINVERSE}    # отменить реверсию

BREAK='\033[m'       #  ${BREAK}    # все атрибуты по умолчанию

NORMAL='\033[0m'      #  ${NORMAL}    # все атрибуты по умолчанию



# Цвет текста: 

BLACK='\033[0;30m'     #  ${BLACK}    # чёрный цвет знаков

RED='\033[0;31m'       #  ${RED}      # красный цвет знаков

GREEN='\033[0;32m'     #  ${GREEN}    # зелёный цвет знаков

YELLOW='\033[0;33m'     #  ${YELLOW}    # желтый цвет знаков

BLUE='\033[0;34m'       #  ${BLUE}      # синий цвет знаков

MAGENTA='\033[0;35m'     #  ${MAGENTA}    # фиолетовый цвет знаков

CYAN='\033[0;36m'       #  ${CYAN}      # цвет морской волны знаков

GRAY='\033[0;37m'       #  ${GRAY}      # серый цвет знаков



# Цветом текста (жирным) (bold) :

DEF='\033[0;39m'       #  ${DEF}

DGRAY='\033[1;30m'     #  ${DGRAY}

LRED='\033[1;31m'       #  ${LRED}

LGREEN='\033[1;32m'     #  ${LGREEN}

LYELLOW='\033[1;33m'     #  ${LYELLOW}

LBLUE='\033[1;34m'     #  ${LBLUE}

LMAGENTA='\033[1;35m'   #  ${LMAGENTA}

LCYAN='\033[1;36m'     #  ${LCYAN}

WHITE='\033[1;37m'     #  ${WHITE}



# Цвет фона 

BGBLACK='\033[40m'     #  ${BGBLACK}

BGRED='\033[41m'       #  ${BGRED}

BGGREEN='\033[42m'     #  ${BGGREEN}

BGBROWN='\033[43m'     #  ${BGBROWN}

BGBLUE='\033[44m'     #  ${BGBLUE}

BGMAGENTA='\033[45m'     #  ${BGMAGENTA}

BGCYAN='\033[46m'     #  ${BGCYAN}

BGGRAY='\033[47m'     #  ${BGGRAY}

BGDEF='\033[49m'      #  ${BGDEF}

#tput sgr0    Return default color 

#Read variable from keypad
#read B #wait enter, result save to B
#read -n 1 B #wait enter one symbol
#read -t 1 -n 1 B #wait enter one symbol by one second

script_path=$PWD

doc_dir="doc"
build_dir="build_doc"

echo -en "${BOLD}${BGDEF}${LBLUE}Current path:${NORMAL} "$PWD"\n"
cd ..
echo -en "${BOLD}${BGDEF}${LBLUE}Project root path:${NORMAL} "$PWD"\n"

#echo -e "${BOLD}${BGDEF}${LGREEN} Меню DNS323 ${BLINK}"

if [ -d $build_dir ]; then
    echo -en "${BOLD}${BGDEF}${LGREEN}Build directory found. Entering build directory\n${NORMAL}"
    cd $build_dir
    build_dir=$PWD
    echo -en "${BOLD}${BGDEF}${LBLUE}Entering directory:${NORMAL} "$PWD"\n"
else
    echo -en "${BOLD}${BGDEF}${LGREEN}Find build directory not found${NORMAL}\n"
    echo -en "${BOLD}${BGDEF}${LGREEN}Create build directory${NORMAL}\n"
    mkdir $build_dir  
    echo -en "${BOLD}${BGDEF}${LBLUE}Entering directory:${NORMAL} "$PWD"\n"
    cd $build_dir 
    build_dir=$PWD
fi   

cd ../

if [ -d $doc_dir ]; then
    echo -en "${BOLD}${BGDEF}${LGREEN}Build directory found. Entering build directory\n${NORMAL}"
    cd $doc_dir
    echo -en "${BOLD}${BGDEF}${LBLUE}Entering directory:${NORMAL} "$PWD"\n"
else
    echo -en "${BOLD}${BGDEF}${LGREEN}Error not found doc: ${doc_dir}${NORMAL}\n"
    exit
fi   

tput sgr0 

echo -en "${BOLD}${BGDEF}${LBLUE}Current path:${NORMAL} "$PWD"\n"

echo -en "${BOLD}${BGDEF}${LBLUE}Run pdflatex${NORMAL}\n"

tex_files=$(find ./ -iname "*.tex")

for var in $tex_files;
do
	tex_name="$var"
	tex_name="$(tr -d ' ' <<< "${tex_name/NEEDED}")"
	echo -en "Tex file name: ${BOLD}${BGDEF}${LMAGENTA}"$tex_name"\n${NORMAL}"
	tex_names+=($tex_name)
done

for tex_name in "${tex_names[@]}";
do
	echo -en "Compile tex file: ${BOLD}${BGDEF}${LMAGENTA}"$tex_name"\n${NORMAL}"
    pdflatex -synctex=1 -interaction=nonstopmode -output-directory=${build_dir} $tex_name
done

cd $build_dir

rm *.aux
rm *.log
rm *.out
rm *.synctex.*
rm *.toc
