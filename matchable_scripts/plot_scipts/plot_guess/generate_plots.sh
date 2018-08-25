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
  echo -e ${BYELLOW}"Please specify the working directory and the output directory for the plots"
  echo -e ${BYELLOW}"Usage: <generate_plots> <path_to_working_directory> <path_where_to_save_plots>"${NC}
  exit 1
fi


#ia preparing all the damn directories 
pwd=`pwd`

target_dir=$1
#ia start listing all the files in the target directory
directories=($(ls ${target_dir}))
#ia plots directory
plots_dir=$2

#ia directories for the stats - in order to easy plot them with gnuplot
plots_directory=${plots_dir}/guess_comparison
if [ -d ${plots_directory} ]; then
    echo -e ${BRED}cleaning plots directory${NC}
    rm -rf ${plots_directory}
fi
mkdir ${plots_directory}

#ia print out the all the shit
echo -e current directory : ${UCYAN}$pwd${NC}
echo -e working directory : ${UCYAN}${target_dir}${NC}
echo -e plots directory : ${UCYAN}${plots_directory}${NC}
echo -e creating plots : ${UCYAN}guess comparison${NC}
cd ${target_dir}
echo $'\n'


#ia for each directory - aka each dataset
for d in "${directories[@]}"; do
  if [ -f ${d} ]; then
      echo -e ${RED}skip ${d}${NC}
      continue
  fi

  echo -e processing dataset:${UCYAN}${d}${NC}

  #ia create directory for the current dataset
  dataset_plots_directory=${plots_directory}/${d}
  if [ -d ${dataset_plots_directory} ]; then
      echo -e ${BRED}cleaning dataset_plots_directory${NC}
      rm -rf ${dataset_plots_directory}
  fi
  mkdir ${dataset_plots_directory}

  echo -e saving plots for current dataset in folder : ${UCYAN}${dataset_plots_directory}${NC}

  # call the damn plotting script <ds_name>
  gnuplot -e "dataset='${d}'; output_directory='${dataset_plots_directory}'" ${pwd}/plot_high.plt 
  echo $'\n'

  gnuplot -e "dataset='${d}'; output_directory='${dataset_plots_directory}'" ${pwd}/plot_mid.plt 
  echo $'\n'

  gnuplot -e "dataset='${d}'; output_directory='${dataset_plots_directory}'" ${pwd}/plot_low.plt 
  echo $'\n'

  gnuplot -e "dataset='${d}'; output_directory='${dataset_plots_directory}'" ${pwd}/plot_rot.plt 
  echo $'\n'
  
  echo -e ${UGREEN}finished this folder${NC}
  echo $'\n'
done

echo $'\n'
echo $'\n'
echo $'\n'
echo -e ${BGREEN}finished plots guess comparison${NC}
cd $pwd
