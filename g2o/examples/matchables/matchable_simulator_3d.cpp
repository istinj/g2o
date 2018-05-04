#include <iostream>
#include <fstream>

#include "matchable_world_simulator.h"
// #include "matchable_world.h"

//ia include g2o core stuff
#include "g2o/stuff/command_args.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"

using namespace std;
using namespace g2o;
using namespace matchables;

int main(int argc, char** argv) {

  //ia simulator paramers
  CommandArgs arg;
  int num_poses;
  float resolution;
  float sense_radius;
  std::string output_filename;
  std::vector<int> world_size;
  //ia factors: from->to
  bool has_point_point_factor;
  bool has_line_line_factor;
  bool has_plane_plane_factor;
  bool has_line_point_factor;
  bool has_plane_line_factor;
  bool has_plane_point_factor;

  arg.param("numPoses", num_poses, 0, "number of robot poses");
  arg.param("worldSize", world_size, std::vector<int>(), "world height and width separated by a semicolumn, e.g. \"7;5\". Default is \"10;10\"");
  arg.param("resolution", resolution, 1.0f, "resolution of the world grid");
  arg.param("senseRadius", sense_radius, 2.0f, "sensing range of the robot");
  arg.param("hasPtPt", has_point_point_factor, false, "point-point factors enabled");
  arg.param("hasLnLn", has_line_line_factor, false, "line-line factors enabled");
  arg.param("hasPlPl", has_plane_plane_factor, false, "plane-plane factors enabled");
  arg.param("hasLnPt", has_line_point_factor, false, "line-point factors enabled");
  arg.param("hasPlLn", has_plane_line_factor, false, "plane-line factors enabled");
  arg.param("hasPlPt", has_plane_point_factor, false, "plane-point factors enabled");
  arg.paramLeftOver("graph-output", output_filename, "", "output of the generator", true);
  arg.parseArgs(argc, argv);

  if (!output_filename.size())
    throw std::runtime_error("no graph-output specified");
  
  if (!world_size.size()) {
    std::cerr << "using default world size" << std::endl;
    world_size.push_back(10);
    world_size.push_back(10);
  }

  g2o::SparseOptimizer opt;

  MatchableWorld* world = new MatchableWorld();
  world->setResolution(resolution);
  world->setHeight(world_size[0]);
  world->setWidth(world_size[1]);
  world->createGrid();
  world->removeWalls(num_poses);

  WorldSimulator ws;
  ws.setNumPoses(num_poses);
  ws.setSenseRadius(sense_radius);
  ws.setVertices(&(opt.vertices()));
  ws.setEdges(&(opt.edges()));
  ws.setWorld(world);

  ws.init();
  
  ws.compute();

  opt.save(output_filename.c_str());
  
  std::cerr << "output graph saved in " << output_filename << std::endl;

  delete world;
  return 0;
}
