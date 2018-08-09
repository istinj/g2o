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


for f in "${files[@]}"; do
  if [ -d ${f} ]; then
      continue
  fi
  echo -e processing file: ${UYELLOW}${f}${NC}
  # get the damn name without extension
  f_base=${f##*/}
  f_prefix=${f_base%.*}

  # we create a directory for each file, so that plot can be done easily
  file_directory=${f_prefix}
  if [ -d ${file_directory} ]; then
      echo -e ${BRED}cleaning file directory${NC}
      rm -rf ${file_directory}/
  fi
  mkdir ${file_directory}/


  # low noise
  low_noise_file=${file_directory}/${f_prefix}_low.g2o
  echo -e output graph: ${UCYAN}${low_noise_file}${NC}
  ${G2O_ROOT}/bin/noise_adder3d -noiseRotation "0.0001;0.0001;0.001" -noiseTranslation "0.01;0.01;0.001" -o ${low_noise_file} ${f}
  echo $'\n'

  
  # mid noise
  mid_noise_file=${file_directory}/${f_prefix}_mid.g2o
  echo -e output graph: ${UCYAN}${mid_noise_file}${NC}
  ${G2O_ROOT}/bin/noise_adder3d -noiseRotation "0.0001;0.0001;0.01" -noiseTranslation "0.1;0.1;0.01" -o ${mid_noise_file} ${f}
  echo $'\n'


  # high noise
  high_noise_file=${file_directory}/${f_prefix}_high.g2o
  echo -e output graph: ${UCYAN}${high_noise_file}${NC}
  ${G2O_ROOT}/bin/noise_adder3d -noiseRotation "0.0001;0.0001;0.1" -noiseTranslation "0.5;0.5;0.01" -o ${high_noise_file} ${f}
  echo $'\n'


  # rot noise
  rot_noise_file=${file_directory}/${f_prefix}_rot.g2o
  echo -e output graph: ${UCYAN}${rot_noise_file}${NC}
  ${G2O_ROOT}/bin/noise_adder3d -noiseRotation "0.001;0.001;0.1" -noiseTranslation "0.001;0.001;0.001" -o ${rot_noise_file} ${f}
  echo $'\n'
  
done

echo -e ${BGREEN}finish${NC}


echo exit
echo $'\n'


