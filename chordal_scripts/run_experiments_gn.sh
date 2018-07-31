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

REVRED='\033[7;31m'
REVCYAN='\033[7;36m'
REVYELLOW='\033[7;33m'
REVBLUE='\033[7;34m'
REVGREEN='\033[7;32m'

NC='\033[0m' # No Color

target_dir=$1
directories=($(ls ${target_dir}))

pwd=`pwd`

echo -e G2O_ROOT: ${UCYAN}${G2O_ROOT}${NC}
echo -e current directory: ${UCYAN}$pwd${NC}
echo -e working directory : ${UCYAN}${target_dir}${NC}
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

  echo -e processing folder: ${REVCYAN}${d}${NC}

  ${pwd}/run_gn_no_guess.sh ${d}
  # read -p "press any key to continue"
  
  ${pwd}/run_gn_spanning.sh ${d}
  # read -p "press any key to continue"
  
  ${pwd}/run_gn_odom.sh ${d}
  # read -p "press any key to continue"

  echo -e ${REVGREEN}finished this folder${NC}
  echo $'\n'
done

echo $'\n'
echo $'\n'
echo $'\n'
echo -e ${REVGREEN}FINISHED EXPERIMENTS GAUSS-NEWTON NO KERNEL${NC}
cd $pwd
