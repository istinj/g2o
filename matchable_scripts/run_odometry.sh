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


# ia generate stats directory
output_directory=output_odometry
if [ -d ${output_directory} ]; then
  echo -e ${BRED}cleaning ouput directory${NC}
  rm -rf ${output_directory}
fi
mkdir ${output_directory}
for f in "${files[@]}"; do
  #ia skip directories
  if [ -d ${f} ]; then
      continue
  fi
  
  echo -e processing file: ${UCYAN}${f}${NC}
  
  # get the damn name without extension
  f_base=${f##*/}
  f_prefix=${f_base%.*}
  output_file=${output_directory}/${f_prefix}_output.g2o
  stats_file=${output_directory}/${f_prefix}.stats
  echo -e output graph: ${UYELLOW}${output_file}${NC}
  echo -e stats file: ${UYELLOW}${stats_file}${NC}


  ${G2O_ROOT}/bin/g2o -v -i 500 -guessOdometry -solver lm_var_cholmod -stats ${stats_file} -o ${output_file} ${f}
  echo $'\n'
done

echo $'\n'
echo -e ${BGREEN}done${NC}

cd $pwd
echo exit



