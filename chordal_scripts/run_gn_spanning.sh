#!/bin/bash

if test "$#" -ne 1; then
  echo "Please specify the working directory"
  exit 1
fi

# COLORSSSS
RED='\033[0;31m'
CYAN='\033[0;36m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
GREEN='\033[0;32m'

BRED='\033[1;31m'
BCYAN='\033[1;36m'
BYELLOW='\033[1;33m'
BBLUE='\033[1;34m'
BGREEN='\033[1;32m'

URED='\033[4;31m'
UCYAN='\033[4;36m'
UYELLOW='\033[4;33m'
UBLUE='\033[4;34m'
UGREEN='\033[4;32m'

NC='\033[0m' # No Color

target_dir=$1
files=($(ls ${target_dir}))

pwd=`pwd`

echo -e G2O_ROOT: ${UCYAN}${G2O_ROOT}${NC}
echo -e current directory: ${UCYAN}$pwd${NC}
echo -e working directory : ${UCYAN}${target_dir}${NC}
cd ${target_dir}
echo $'\n'

output_directory=output_gn_spanning
if [ -d ${output_directory} ]; then
    echo -e ${BRED}cleaning standard output directory${NC}
    rm -rf ${output_directory}
fi
mkdir ${output_directory}

output_directory_chordal=chordal/output_gn_spanning
if [ -d ${output_directory_chordal} ]; then
    echo -e ${BRED}cleaning chordal output directory${NC}
    rm -rf ${output_directory_chordal}
fi
mkdir -p ${output_directory_chordal}

# output_directory_compare=compare
# if [ -d ${output_directory_compare} ]; then
#     echo -e ${BRED}cleaning compare stats directory${NC}
#     rm -rf ${output_directory_compare}
# fi
# mkdir -p ${output_directory_compare}

#ia for each damn file in the directory 
for f in "${files[@]}"; do
  if [ -d ${f} ]; then
      echo -e ${RED}skip directory${NC}
      continue
  fi

  echo -e processing file: ${UCYAN}${f}${NC}
  # get the damn name without extension
  f_base=${f##*/}
  f_prefix=${f_base%.*}

  #ia output files
  output_file_standard=${output_directory}/${f_prefix}_output.g2o
  output_file_chordal=${output_directory_chordal}/${f_prefix}_output.g2o

  stats_file_standard=${output_directory}/${f_prefix}.stats
  stats_file_chordal=${output_directory_chordal}/${f_prefix}.stats
  # stats_file_compare=${output_directory_compare}/${f_prefix}.stats

  # echo -e stats files: ${UYELLOW}${stats_file_standard}${NC} and ${UYELLOW}${stats_file_chordal}${NC} and ${UYELLOW}${stats_file_compare}${NC}
  echo -e stats files: ${UYELLOW}${stats_file_standard}${NC} and ${UYELLOW}${stats_file_chordal}${NC}
  echo -e output files: ${UYELLOW}${output_file_standard}${NC} and ${UYELLOW}${output_file_chordal}${NC}

  #ia process the standard and the chordal one
  echo -e ${YELLOW}standard${NC}
  ${G2O_ROOT}/bin/g2o -v -i 100 -guess -solver gn_fix6_3_cholmod -guess -stats ${stats_file_standard} -o ${output_file_standard} ${f}
  
  # echo $'\n'
  # echo -e ${YELLOW}chordal${NC}
  # ${G2O_ROOT}/bin/g2o -v -i 100 -guess -solver gn_fix6_3_cholmod -stats ${stats_file_chordal} -o ${output_file_chordal} chordal/${f}
  # echo $'\n'
  # echo -e ${YELLOW}compare${NC}
  # ${G2O_ROOT}/bin/chordal_comparator -i 100 -compareStats ${stats_file_compare} -otherGraph ${f} chordal/${f}
  
  echo $'\n'
  echo -e ${YELLOW}chordal${NC}
  ${G2O_ROOT}/bin/chordal_comparator -i 100 -solver gn_fix6_3_cholmod -guess -o ${output_file_chordal} -compareStats ${stats_file_chordal} -otherGraph ${f} chordal/${f}
  
done

echo -e ${BGREEN}finish${NC}
cd $pwd
echo exit
echo $'\n'
