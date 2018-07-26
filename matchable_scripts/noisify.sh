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

# ia generate low noise graphs
echo -e ${BCYAN}low noise${NC}
if [ -d low_noise ]; then
  echo -e ${BRED}cleaning noise directory${NC}
  rm -rf low_noise/
fi
mkdir low_noise
for f in "${files[@]}"; do
  if [ -d ${f} ]; then
      continue
  fi
  echo -e processing file: ${UCYAN}${f}${NC}
  # get the damn name without extension
  f_base=${f##*/}
  f_prefix=${f_base%.*}
  output_file=low_noise/${f_prefix}.g2o
  echo output graph low_noise/${f_prefix}.g2o
  
  ${G2O_ROOT}/bin/g2o_matchable_noise_adder -pointNoise "0.005;0.005;0.05" -normalNoise "0.001;0.001" -translationNoise "0.01;0.01;0.001" -rotationNoise "0.001;0.001;0.005" -o ${output_file} ${f}
  echo $'\n'
done
echo $'\n'

# ia generate mid noise graphs
echo -e ${BCYAN}mid noise${NC}
if [ -d mid_noise ]; then
    echo -e ${BRED}cleaning noise directory${NC}
    rm -rf mid_noise/
fi
mkdir mid_noise
for f in "${files[@]}"; do
  if [ -d ${f} ]; then
      continue
  fi
  echo -e processing file: ${UCYAN}${f}${NC}
  # get the damn name without extension
  f_base=${f##*/}
  f_prefix=${f_base%.*}
  output_file=mid_noise/${f_prefix}.g2o
  echo output graph: ${output_file}
  
  ${G2O_ROOT}/bin/g2o_matchable_noise_adder -pointNoise "0.05;0.05;0.1" -normalNoise "0.01;0.01" -translationNoise "0.1;0.1;0.01" -rotationNoise "0.01;0.01;0.05" -o ${output_file} ${f}
  echo $'\n'
done
echo $'\n'


# ia generate mid noise graphs
echo -e ${BCYAN}high noise${NC}
if [ -d high_noise ]; then
    echo -e ${BRED}cleaning noise directory${NC}
    rm -rf high_noise/
fi
mkdir high_noise
for f in "${files[@]}"; do
  if [ -d ${f} ]; then
      continue
  fi
  echo -e processing file: ${UCYAN}${f}${NC}
  # get the damn name without extension
  f_base=${f##*/}
  f_prefix=${f_base%.*}
  output_file=high_noise/${f_prefix}.g2o
  echo output graph: ${output_file}
  
  ${G2O_ROOT}/bin/g2o_matchable_noise_adder -pointNoise "0.5;0.5;0.1" -normalNoise "0.1;0.1" -translationNoise "1.0;1.0;0.01" -rotationNoise "0.01;0.01;0.1" -o ${output_file} ${f}
  echo $'\n'
done
echo done

cd $pwd
echo $'\n'
echo exit



