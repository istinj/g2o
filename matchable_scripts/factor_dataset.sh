#!/bin/bash

if test "$#" -ne 1; then
    echo "Please specify the output directory"
    exit 1
fi

target_dir=$1

pwd=`pwd`

echo G2O_ROOT: ${G2O_ROOT}
echo current directory: $pwd
echo output directory : ${target_dir}

cd ${G2O_ROOT}/bin

# homogenous factors
./g2o_matchable_simulator3d -numPoses 100 -numPoints 500 -numLines 100 -numPlanes 50 -hasPtPt ${target_dir}/graph_0_pt.g2o
./g2o_matchable_simulator3d -numPoses 100 -numPoints 500 -numLines 100 -numPlanes 50 -hasLnLn ${target_dir}/graph_0_ln.g2o
./g2o_matchable_simulator3d -numPoses 100 -numPoints 500 -numLines 100 -numPlanes 50 -hasPlPl ${target_dir}/graph_0_pl.g2o
./g2o_matchable_simulator3d -numPoses 100 -numPoints 500 -numLines 100 -numPlanes 50 -hasPtPt -hasLnLn -hasPlPl ${target_dir}/graph_0_homogeneous.g2o

# homogenous factor and their respective lower level ones
./g2o_matchable_simulator3d -numPoses 100 -numPoints 500 -numLines 100 -numPlanes 50 -hasLnLn -hasLnPt ${target_dir}/graph_0_ln_lnpt.g2o
./g2o_matchable_simulator3d -numPoses 100 -numPoints 500 -numLines 100 -numPlanes 50 -hasPlPl -hasPlLn ${target_dir}/graph_0_pl_plln.g2o
./g2o_matchable_simulator3d -numPoses 100 -numPoints 500 -numLines 100 -numPlanes 50 -hasPlPl -hasPlLn -hasPlPt ${target_dir}/graph_0_pl_plln_plpt.g2o
./g2o_matchable_simulator3d -numPoses 100 -numPoints 500 -numLines 100 -numPlanes 50 -hasLnPt -hasPlLn -hasPlPt ${target_dir}/graph_0_inhomogenous.g2o

# all factors
./g2o_matchable_simulator3d -numPoses 100 -numPoints 500 -numLines 100 -numPlanes 50 -hasAllFactors ${target_dir}/graph_0_all.g2o

cd $pwd

echo $'\n'
echo $'\n'
echo done

