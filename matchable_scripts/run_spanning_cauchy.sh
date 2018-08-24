#!/bin/bash

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


if test "$#" -ne 2; then
  echo -e ${BYELLOW}"Please specify the working directory and the output directory for the statics"
  echo -e ${BYELLOW}"Usage: <run_experiments> <path_to_working_directory> <path_where_to_save_stats>"${NC}
  exit 1
fi

target_dir=$1
files=($(ls ${target_dir}))
pwd=`pwd`

stats_dir=$2

iterations=100

#ia directories for the stats - in order to easy plot them with gnuplot
statics_directory=${stats_dir}/stats_spanning
if [ -d ${statics_directory} ]; then
    echo -e ${BRED}cleaning statics directory${NC}
    rm -rf ${statics_directory}
fi
mkdir ${statics_directory}


#ia print out the all the shit
echo -e G2O_ROOT: ${UCYAN}${G2O_ROOT}${NC}
echo -e current directory : ${UCYAN}$pwd${NC}
echo -e working directory : ${UCYAN}${target_dir}${NC}
echo -e stats directory : ${UCYAN}${statics_directory}${NC}
echo -e running: ${UCYAN}Levemberg-Marquardt with Cauchy kernel spanning${NC}
cd ${target_dir}
echo $'\n'

# ia generate output directory
output_directory=output_spanning
if [ -d ${output_directory} ]; then
  echo -e ${BRED}cleaning ouput directory${NC}
  rm -rf ${output_directory}
fi
mkdir ${output_directory}

for f in "${files[@]}"; do
  #ia skip directories
  if [ -d ${f} ]; then
      echo -e ${RED}skip directory${NC}
      continue
  fi
  
  echo -e processing file: ${UCYAN}${f}${NC}
  
  # get the damn name without extension
  f_base=${f##*/}
  f_prefix=${f_base%.*}
  
  output_file=${output_directory}/${f_prefix}_output.g2o
  stats_file=${statics_directory}/${f_prefix}.stats
  summary_file=${statics_directory}/${f_prefix}.summary
  echo -e output graph: ${UYELLOW}${output_file}${NC}
  echo -e stats file: ${UYELLOW}${stats_file}${NC}
  echo -e summary file: ${UYELLOW}${summary_file}${NC}


  ${G2O_ROOT}/bin/g2o -v -i ${iterations} -guess -solver lm_var_cholmod -robustKernel Cauchy -robustKernelWidth 1.0 -stats ${stats_file} -summary ${summary_file} -o ${output_file} ${f}
  echo $'\n'
done

echo $'\n'
echo -e ${BGREEN}done${NC}

cd $pwd
echo exit
echo $'\n'



