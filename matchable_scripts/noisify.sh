#!/bin/bash

if test "$#" -ne 1; then
  echo "Please specify the working directory"
  exit 1
fi

target_dir=$1
files=($(ls ${target_dir}))

pwd=`pwd`

echo G2O_ROOT: ${G2O_ROOT}
echo current directory: $pwd
echo output directory : ${target_dir}
cd ${target_dir}
echo $'\n'


# ia generate low noise graphs
echo low noise
if [ -d low_noise ]; then
  echo cleaning noise directory
  rm -rf low_noise/
fi
mkdir low_noise
for f in "${files[@]}"; do
  if [ -d ${f} ]; then
      continue
  fi
  echo processing file ${f}
  # get the damn name without extension
  f_base=${f##*/}
  f_prefix=${f_base%.*}
  echo output graph low_noise/${f_prefix}_N.g2o
  
  ${G2O_ROOT}/bin/g2o_matchable_noise_adder -pointNoise "0.005;0.005;0.005" -normalNoise "0.001;0.001" -translationNoise "0.005;0.005;0.005" -rotationNoise "0.001;0.001;0.001" -o low_noise/${f_prefix}_N.g2o ${f}
  echo $'\n'
done
echo $'\n'

# ia generate mid noise graphs
echo mid noise
if [ -d mid_noise ]; then
    echo cleaning noise directory
    rm -rf mid_noise/
fi
mkdir mid_noise
for f in "${files[@]}"; do
  if [ -d ${f} ]; then
      continue
  fi
  echo processing file ${f}
  # get the damn name without extension
  f_base=${f##*/}
  f_prefix=${f_base%.*}
  echo output graph mid_noise/${f_prefix}_N.g2o
  
  ${G2O_ROOT}/bin/g2o_matchable_noise_adder -pointNoise "0.01;0.01;0.01" -normalNoise "0.005;0.005" -translationNoise "0.01;0.01;0.01" -rotationNoise "0.005;0.005;0.005" -o mid_noise/${f_prefix}_N.g2o ${f}
  echo $'\n'
done
echo $'\n'


# ia generate mid noise graphs
echo high noise
if [ -d high_noise ]; then
  echo cleaning noise directory
  rm -rf high_noise/
fi
mkdir high_noise
for f in "${files[@]}"; do
  if [ -d ${f} ]; then
      continue
  fi
  echo processing file ${f}
  # get the damn name without extension
  f_base=${f##*/}
  f_prefix=${f_base%.*}
  echo output graph high_noise/${f_prefix}_N.g2o
  
  ${G2O_ROOT}/bin/g2o_matchable_noise_adder -pointNoise "0.1;0.1;0.1" -normalNoise "0.05;0.05" -translationNoise "0.1;0.1;0.1" -rotationNoise "0.05;0.05;0.05" -o high_noise/${f_prefix}_N.g2o ${f}
  echo $'\n'
done
echo done

cd $pwd
echo $'\n'
echo exit



