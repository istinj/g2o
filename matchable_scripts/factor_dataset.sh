#!/bin/bash

if test "$#" -ne 1; then
    echo "Please specify the output directory"
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
pwd=`pwd`

echo -e G2O_ROOT: ${UCYAN}${G2O_ROOT}${NC}
echo -e current directory: ${UCYAN}$pwd${NC}
echo -e output directory : ${UCYAN}${target_dir}${NC}

cd ${G2O_ROOT}/bin

# homogenous factors
./g2o_matchable_simulator3d -numPoses 100 -numPoints 200 -numLines 200 -numPlanes 200 -hasPtPt ${target_dir}/graph_0_pt.g2o
./g2o_matchable_simulator3d -numPoses 100 -numPoints 200 -numLines 200 -numPlanes 200 -hasLnLn ${target_dir}/graph_0_ln.g2o
./g2o_matchable_simulator3d -numPoses 100 -numPoints 200 -numLines 200 -numPlanes 200 -hasPlPl ${target_dir}/graph_0_pl.g2o
./g2o_matchable_simulator3d -numPoses 100 -numPoints 200 -numLines 200 -numPlanes 200 -hasPtPt -hasLnLn -hasPlPl ${target_dir}/graph_0_homogeneous.g2o

# homogenous factor and their respective lower level ones
./g2o_matchable_simulator3d -numPoses 100 -numPoints 200 -numLines 200 -numPlanes 200 -hasLnLn -hasLnPt ${target_dir}/graph_0_ln_lnpt.g2o
./g2o_matchable_simulator3d -numPoses 100 -numPoints 200 -numLines 200 -numPlanes 200 -hasPlPl -hasPlLn ${target_dir}/graph_0_pl_plln.g2o
./g2o_matchable_simulator3d -numPoses 100 -numPoints 200 -numLines 200 -numPlanes 200 -hasPlPl -hasPlLn -hasPlPt ${target_dir}/graph_0_pl_plln_plpt.g2o
./g2o_matchable_simulator3d -numPoses 100 -numPoints 200 -numLines 200 -numPlanes 200 -hasLnPt -hasPlLn -hasPlPt ${target_dir}/graph_0_inhomogenous.g2o

# all factors
./g2o_matchable_simulator3d -numPoses 100 -numPoints 200 -numLines 200 -numPlanes 200 -hasAllFactors ${target_dir}/graph_0_all.g2o

echo $'\n'
echo -e ${BGREEN}done${NC}

cd $pwd
echo exit

