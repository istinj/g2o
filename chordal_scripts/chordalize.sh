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
      # if it is a file skip
      continue
  fi

  if [ "${d}" = "chordal_optimum" ]; then
      echo -e ${RED}skip folder ${d}${NC}
      echo $'\n'
      continue
  fi

  echo -e processing folder: ${UCYAN}${d}${NC}
  files=($(ls ${d}))
  cd ${d}

  #ia create a directory for the chordal
  chordal_directory=chordal
  if [ -d ${chordal_directory} ]; then
      echo -e ${BRED}cleaning chordal directory${NC}
      rm -rf ${chordal_directory}
  fi
  mkdir ${chordal_directory}

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
    output_file=${chordal_directory}/${f_prefix}.g2o
    echo -e output graph: ${UYELLOW}${output_file}${NC}

    #ia default values for the omega conversion
    ${G2O_ROOT}/bin/chordal_converter -condType 0 -omegaTresh 0.1 -o ${output_file} ${f}
  done


  echo -e ${GREEN}finished this folder${NC}
  echo $'\n'
  cd ${target_dir}
done



echo -e ${BGREEN}finish${NC}


echo exit
echo $'\n'
