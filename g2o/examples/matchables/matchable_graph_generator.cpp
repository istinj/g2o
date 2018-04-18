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
  bool has_line_line_factor;
  bool has_plane_plane_factor;
  
  arg.param("numPoses", num_poses, 0, "number of robot poses");
  arg.param("numPoints", num_points, 0, "number of matchable-points in the graph");
  arg.param("numLines", num_lines, 0, "number of matchable-lines in the graph");
  arg.param("numPlanes", num_planes, 0, "number of matchable-planes in the graph");
  arg.param("worldSize", world_size, 10.0, "dimension of the world");
  arg.param("hasPointPointFactor", has_point_point_factor, false, "point-point factors enabled");
  arg.param("hasLineLineFactor", has_line_line_factor, false, "line-line factors enabled");
  arg.param("hasPlanePlaneFactor", has_plane_plane_factor, false, "plane-plane factors enabled");
  // arg.param("applyPerturbation", apply_perturbation, false, "apply a perturbation to the vertices");
  
  arg.paramLeftOver("graph-output", output_filename, "", "output of the generator", true);
  
  arg.parseArgs(argc, argv);

  if (!output_filename.size()) {
    throw std::runtime_error("No output specified");
  } 

  // ia registering all the types from the libraries
  DlWrapper dlTypesWrapper;
  loadStandardTypes(dlTypesWrapper, argc, argv);

  SparseOptimizer opt;

  //ia black magic setup
  MatchableGenerator g;
  g.config().num_poses = num_poses;
  g.config().num_points = num_points;
  g.config().num_lines = num_lines;
  g.config().num_planes = num_planes;
  g.config().world_size = world_size;
  g.config().has_point_factors = has_point_point_factor;
  g.config().has_line_factors = has_line_line_factor;
  g.config().has_plane_factors = has_plane_plane_factor;
  g.setup();

  //ia black magic init
  //ia jesus does not love me so i have to switch to references here
  g.setVertices(&(opt.vertices()));
  g.setEdges(&(opt.edges()));
  g.init();

  //ia black magic compute
  g.compute();

  //ia save graph
  std::cerr << "saving the graph in: " << output_filename << std::endl;
  opt.save(output_filename.c_str());
  
  return 0;
}
