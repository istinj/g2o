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

#ia preparing all the damn directories 
pwd=`pwd`

target_dir=$1
#ia start listing all the files in the target directory
files=($(ls ${target_dir}))

stats_dir=$2

#ia directories for the stats - in order to easy plot them with gnuplot
statics_directory_base=${stats_dir}/stats_odometry
if [ -d ${statics_directory_base} ]; then
    echo -e ${BRED}cleaning statics directory${NC}
    rm -rf ${statics_directory_base}
fi
mkdir ${statics_directory_base}

statics_directory_geodesic=${statics_directory_base}/geodesic
if [ -d ${statics_directory_geodesic} ]; then
    echo -e ${BRED}cleaning statics_directory_geodesic directory${NC}
    rm -rf ${statics_directory_geodesic}
fi
mkdir ${statics_directory_geodesic}

statics_directory_chordal=${statics_directory_base}/chordal
if [ -d ${statics_directory_chordal} ]; then
    echo -e ${BRED}cleaning statics_directory_chordal directory${NC}
    rm -rf ${statics_directory_chordal}
fi
mkdir ${statics_directory_chordal}


#ia print out the all the shit
echo -e G2O_ROOT: ${UCYAN}${G2O_ROOT}${NC}
echo -e current directory : ${UCYAN}$pwd${NC}
echo -e working directory : ${UCYAN}${target_dir}${NC}
echo -e stats directory : ${UCYAN}${statics_directory_base}${NC}
echo -e running: ${UCYAN}Levemberg-Marquardt odometry${NC}
cd ${target_dir}
echo $'\n'


#ia directories for the outputs
output_directory_geodesic=output_odometry
if [ -d ${output_directory_geodesic} ]; then
    echo -e ${BRED}cleaning standard output directory${NC}
    rm -rf ${output_directory_geodesic}
fi
mkdir ${output_directory_geodesic}

output_directory_chordal=chordal/output_odometry
if [ -d ${output_directory_chordal} ]; then
    echo -e ${BRED}cleaning chordal output directory${NC}
    rm -rf ${output_directory_chordal}
fi
mkdir -p ${output_directory_chordal}



#ia now we can start doing something
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
  output_file_geodesic=${output_directory_geodesic}/${f_prefix}_output.g2o
  output_file_chordal=${output_directory_chordal}/${f_prefix}_output.g2o

  stats_file_geodesic=${statics_directory_geodesic}/${f_prefix}.stats
  stats_file_chordal=${statics_directory_chordal}/${f_prefix}.stats

  summary_file_geodesic=${statics_directory_geodesic}/${f_prefix}.summary
  summary_file_chordal=${statics_directory_chordal}/${f_prefix}.summary

  echo -e stats files: ${UYELLOW}${stats_file_geodesic}${NC} and ${UYELLOW}${stats_file_chordal}${NC}
  echo -e summary files: ${UYELLOW}${summary_file_geodesic}${NC} and ${UYELLOW}${summary_file_chordal}${NC}
  echo -e output files: ${UYELLOW}${output_file_geodesic}${NC} and ${UYELLOW}${output_file_chordal}${NC}

  #ia process the standard and the chordal one
  echo -e ${YELLOW}standard${NC}
  ${G2O_ROOT}/bin/g2o_chordal_app -v -i 100 -guessOdometry -solver lm_fix6_3_cholmod -stats ${stats_file_geodesic} -summary ${summary_file_geodesic} -o ${output_file_geodesic} ${f}

  echo $'\n'
  echo -e ${YELLOW}chordal${NC}
  ${G2O_ROOT}/bin/g2o_chordal_geodesic_comparator -i 101 -guessOdometry -solver lm_fix6_3_cholmod -o ${output_file_chordal} -compareStats ${stats_file_chordal} -summary ${summary_file_chordal} -geodesicGraph ${f} chordal/${f}
  
done

echo -e ${BGREEN}finish${NC}
cd $pwd
echo exit
echo $'\n'






