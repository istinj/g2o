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
echo -e working directory : ${UCYAN}${target_dir}${NC}

cd ${target_dir}
echo $'\n'

# ia generate 10000 poses graphs
big_graph_dir=10000_poses
echo -e ${BCYAN}10000_poses graph${NC}
if [ -d ${big_graph_dir} ]; then
  echo -e ${BRED}cleaning ${big_graph_dir} graph directory${NC}
  rm -rf ${big_graph_dir}/
fi
mkdir ${big_graph_dir}

${G2O_ROOT}/bin/g2o_matchable_simulator3d -numPoses 10000 -numPoints 8000 -numLines 5000 -numPlanes 5000 -worldSize "120;120" -hasAllFactors ${big_graph_dir}/graph_0_all.g2o

${G2O_ROOT}/bin/g2o_matchable_simulator3d -numPoses 10000 -numPoints 8000 -numLines 5000 -numPlanes 5000 -worldSize "120;120" -hasPtPt -hasLnLn -hasPlPl ${big_graph_dir}/graph_0_homogeneous.g2o

${G2O_ROOT}/bin/g2o_matchable_simulator3d -numPoses 10000 -numPoses 10000 -numPoints 8000 -numLines 5000 -numPlanes 5000 -worldSize "120;120" -hasLnPt -hasPlLn -hasPlPt ${big_graph_dir}/graph_0_inhomogenous.g2o



# ia generate 1000 poses graphs
mid_graph_dir=1000_poses
echo -e ${BCYAN}1000_poses graph${NC}
if [ -d ${mid_graph_dir} ]; then
  echo -e ${BRED}cleaning ${mid_graph_dir} graph directory${NC}
  rm -rf ${mid_graph_dir}/
fi
mkdir ${mid_graph_dir}

${G2O_ROOT}/bin/g2o_matchable_simulator3d -numPoses 1000 -numPoints 1000 -numLines 1000 -numPlanes 1000 -worldSize "40;40" -hasAllFactors ${mid_graph_dir}/graph_0_all.g2o

${G2O_ROOT}/bin/g2o_matchable_simulator3d -numPoses 1000 -numPoints 1000 -numLines 1000 -numPlanes 1000 -worldSize "40;40" -hasPtPt -hasLnLn -hasPlPl ${mid_graph_dir}/graph_0_homogeneous.g2o

${G2O_ROOT}/bin/g2o_matchable_simulator3d -numPoses 1000 -numPoints 1000 -numLines 1000 -numPlanes 1000 -worldSize "40;40" -hasLnPt -hasPlLn -hasPlPt ${mid_graph_dir}/graph_0_inhomogenous.g2o



# ia generate 100 poses graphs
small_graph_dir=100_poses
echo -e ${BCYAN}100_poses graph${NC}
if [ -d ${small_graph_dir} ]; then
  echo -e ${BRED}cleaning ${small_graph_dir} graph directory${NC}
  rm -rf ${small_graph_dir}/
fi
mkdir ${small_graph_dir}

${G2O_ROOT}/bin/g2o_matchable_simulator3d -numPoses 100 -numPoints 50 -numLines 50 -numPlanes 50 -hasAllFactors ${small_graph_dir}/graph_0_all.g2o

${G2O_ROOT}/bin/g2o_matchable_simulator3d -numPoses 100 -numPoints 50 -numLines 50 -numPlanes 50 -hasPtPt -hasLnLn -hasPlPl ${small_graph_dir}/graph_0_homogeneous.g2o

${G2O_ROOT}/bin/g2o_matchable_simulator3d -numPoses 100 -numPoints 50 -numLines 50 -numPlanes 50 -hasLnPt -hasPlLn -hasPlPt ${small_graph_dir}/graph_0_inhomogenous.g2o


echo $'\n'
echo -e ${BGREEN}done${NC}

cd $pwd
echo exit
