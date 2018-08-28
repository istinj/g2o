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

REVRED='\033[7;31m'
REVCYAN='\033[7;36m'
REVYELLOW='\033[7;33m'
REVBLUE='\033[7;34m'
REVGREEN='\033[7;32m'

NC='\033[0m' # No Color

if test "$#" -ne 2; then
  echo -e ${BYELLOW}"Please specify the working directory AND the output directory for the statics"
  echo -e ${BYELLOW}"Usage: <run_experiments> <path_to_working_directory> <path_where_to_save_stats>"${NC}
  exit 1
fi

target_dir=$1
stats_dir=$2
directories=($(ls ${target_dir}))

pwd=`pwd`

echo -e G2O_ROOT: ${REVCYAN}${G2O_ROOT}${NC}
echo -e current directory: ${REVCYAN}$pwd${NC}
echo -e working directory : ${REVCYAN}${target_dir}${NC}
echo -e statics base directory : ${REVCYAN}${stats_dir}${NC}
cd ${target_dir}
echo $'\n'

#ia for each directory - aka each dataset
for d in "${directories[@]}"; do
  if [ -f ${d} ]; then
      echo -e ${REVRED}skip ${d}${NC}
      continue
  fi

  if [ "${d}" = "chordal_optimum" ]; then
      echo -e ${REVRED}skip folder ${d}${NC}
      echo $'\n'
      continue
  fi

  #ia directories for the stats - in order to easy plot them with gnuplot
  statics_directory_dataset=${stats_dir}/${d}
  if [ -d ${statics_directory_dataset} ]; then
      echo -e ${REVRED}cleaning statics directory${NC}
      rm -rf ${statics_directory_dataset}
  fi
  mkdir ${statics_directory_dataset}


  echo -e processing folder : ${REVCYAN}${d}${NC}
  echo -e saving statics in folder : ${REVCYAN}${statics_directory_dataset}${NC}
  echo $'\n'

  ${pwd}/run_gn_no_guess_dyneps.sh ${d}/ ${statics_directory_dataset}
  # read -p "press any key to continue"

  ${pwd}/run_gn_spanning_dyneps.sh ${d}/ ${statics_directory_dataset}
  # read -p "press any key to continue"

  ${pwd}/run_gn_odom_dyneps.sh ${d}/ ${statics_directory_dataset}
  # read -p "press any key to continue"

  echo -e ${REVGREEN}finished this folder${NC}
  echo $'\n'
done

echo $'\n'
echo $'\n'
echo $'\n'
echo -e ${REVGREEN}FINISHED EXPERIMENTS GAUSS-NEWTON NO KERNEL${NC}
cd $pwd
