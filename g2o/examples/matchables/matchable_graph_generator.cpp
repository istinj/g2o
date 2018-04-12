#include <iostream>
#include <fstream>
#include <vector>

//ia include fancy colors
#include "g2o/stuff/color_macros.h"

//ia include types
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/matchables3d/types_matchables.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"

//ia include g2o core stuff
#include "g2o/stuff/command_args.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"

//ia include dynamic library things
#include "g2o/apps/g2o_cli/dl_wrapper.h"
#include "g2o/apps/g2o_cli/output_helper.h"
#include "g2o/apps/g2o_cli/g2o_common.h"

#include <Eigen/Core>
#include <Eigen/StdVector>

//ia ou stuff
#include "matchable_generator.h"

using namespace std;
using namespace g2o;
using namespace matchables;


int main(int argc, char** argv) {

  // ia command line parsing
  CommandArgs arg;
  
  string output_filename;
  int num_poses;
  int num_points;
  int num_lines;
  int num_planes;
  float world_size;
  bool has_point_point_factor;
  bool has_line_point_factor;
  bool has_plane_point_factor;
  bool has_line_line_factor;
  bool has_plane_line_factor;
  bool has_plane_plane_factor;
  bool apply_perturbation;
  
  arg.param("numPoses", num_poses, 1, "number of robot poses");
  arg.param("numPoints", num_points, 1, "number of matchable-points in the graph");
  arg.param("numLines", num_lines, 0, "number of matchable-lines in the graph");
  arg.param("numPlanes", num_planes, 0, "number of matchable-planes in the graph");
  arg.param("worldSize", world_size, 10.0, "dimension of the world");
  arg.param("hasPointPointFactor", has_point_point_factor, false, "point-point factors enabled");
  arg.param("hasLinePointFactor", has_line_point_factor, false, "line-point factors enabled");
  arg.param("hasPlanePointFactor", has_plane_point_factor, false, "plane-point factors enabled");
  arg.param("hasLineLineFactor", has_line_line_factor, false, "line-line factors enabled");
  arg.param("hasPlaneLineFactor", has_plane_line_factor, false, "plane-line factors enabled");
  arg.param("hasPlanePlaneFactor", has_plane_plane_factor, false, "plane-plane factors enabled");
  arg.param("applyPerturbation", apply_perturbation, false, "apply a perturbation to the vertices");
  
  arg.paramLeftOver("graph-output", output_filename, "", "output of the generator", true);
  
  arg.parseArgs(argc, argv);

  if (!output_filename.size()) {
    throw std::runtime_error("No output specified");
  } 

  // ia registering all the types from the libraries
  g2o::DlWrapper dlTypesWrapper;
  g2o::loadStandardTypes(dlTypesWrapper, argc, argv);

  //ia generate constraints vector
  matchables::IntVector constraints(6, 0);
  constraints[0] = has_point_point_factor; //point-point
  constraints[1] = has_line_point_factor; //line-point
  constraints[2] = has_line_line_factor; //line-line
  constraints[3] = has_plane_point_factor; //plane-point
  constraints[4] = has_plane_line_factor; //plane-line
  constraints[5] = has_plane_plane_factor; //plane-plane

  //ia setup the generator
  matchables::GraphGenerator g;
  g.setConstraints(constraints);
  g.setNumPoses(num_poses);
  g.setNumPoints(num_points);
  g.setNumLines(num_lines);
  g.setNumPlanes(num_planes);
  g.setWorldSize(world_size);

  g.init();

  Isometry3Vector poses;
  MatchableVector landmarks;
  MatchableMatrix7PairVector measurements;
  IntIntPairVector landmark_associations;
  
  g.generatePoses(poses);
  g.generateLandmarks(landmarks);
  g.generateMeasurements(measurements,
                         landmark_associations,
                         landmarks,
                         poses);

  if (apply_perturbation)
    g.applyPerturbationToData(landmarks, poses);
    
  std::ofstream file(output_filename.c_str());  
  int id=0;
  
  for(size_t i=0; i<poses.size(); ++i){
    const Isometry3 &pose = poses[i];
    file << "VERTEX_SE3:CHORD " << id << " ";
    const Vector7 v = internal::toVectorQT(pose);
    for(size_t j=0; j<7; ++j)
      file << v[j] << " ";
    file << std::endl;
    ++id;
  }


  for(size_t i=0; i<landmarks.size(); ++i){
    const Matchable &landmark = landmarks[i];
    file << "VERTEX_MATCHABLE " << id << " ";
    const Vector13 v = landmark.toVector();
    for(size_t j=0; j<13; ++j)
      file << v[j] << " ";
    file << std::endl;
    ++id;
  }
  
  for(size_t i=0; i<measurements.size(); ++i){
    const MatchableMatrix7Pair& measurement = measurements[i];
    const Matchable& matchable = measurement.first;
    const Matrix7& omega = measurement.second;
    file << "EDGE_SE3_MATCHABLE " 
          << landmark_associations[i].first << " "
          << g.numPoses()+landmark_associations[i].second << " ";
    
    Vector13 v = matchable.toVector();
    for(size_t j=0; j<13; ++j)
      file << v[j] << " ";

    for (int r = 0; r < omega.rows(); ++r) {
      for (int c = r; c < omega.cols(); ++c) {
        file << omega(r,c) << " ";
      }
    }
    
    file << std::endl;
    ++id;
  }

  file.close();
  return 0;
}
