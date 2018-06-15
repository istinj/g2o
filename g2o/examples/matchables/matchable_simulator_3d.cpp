#include <iostream>
#include <fstream>

#include "matchable_world_simulator.h"

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
  int num_points;
  int num_lines;
  int num_planes;
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

  bool simulator_has_noise;
  std::vector<int> normal_noise;
  std::vector<int> translational_noise;

  arg.param("numPoses", num_poses, 0, "number of robot poses");
  arg.param("worldSize", world_size, std::vector<int>(), "world height and width separated by a semicolumn, e.g. \"7;5\". Default is \"10;10\"");
  arg.param("numPoints", num_points, 0, "number of point landmarks in the world");
  arg.param("numLines", num_lines, 0, "number of line landmarks in the world");
  arg.param("numPlanes", num_planes, 0, "number of plane landmarks in the world");
  arg.param("resolution", resolution, 1.0f, "resolution of the world grid");
  arg.param("senseRadius", sense_radius, 2.0f, "sensing range of the robot");
  arg.param("hasPtPt", has_point_point_factor, false, "point-point factors enabled");
  arg.param("hasLnLn", has_line_line_factor, false, "line-line factors enabled");
  arg.param("hasPlPl", has_plane_plane_factor, false, "plane-plane factors enabled");
  arg.param("hasLnPt", has_line_point_factor, false, "line-point factors enabled");
  arg.param("hasPlLn", has_plane_line_factor, false, "plane-line factors enabled");
  arg.param("hasPlPt", has_plane_point_factor, false, "plane-point factors enabled");
  arg.param("normalNoise", normal_noise, std::vector<int>(), "matchable normal noise as <ny,nz>. default is \"0;0\"");
  arg.param("translationalNoise", translational_noise, std::vector<int>(), "translational noise as <nx,ny,nz>. default is \"0;0;0\"");
  arg.paramLeftOver("graph-output", output_filename, "", "output of the generator", true);
  arg.parseArgs(argc, argv);

  if (!output_filename.size())
    throw std::runtime_error("no graph-output specified");
  
  if (!world_size.size()) {
    world_size.push_back(10);
    world_size.push_back(10);
  }

  Vector2 n_noise_param = Vector2::Zero();
  Vector3 t_noise_param = Vector3::Zero();
  if (!normal_noise.size() && !translational_noise.size()) {
    simulator_has_noise = false;
  } else {
    simulator_has_noise = true;
    for (int i = 0; i < !translational_noise.size(); ++i)
      t_noise_param[i] = translational_noise[i];
    for (int i = 0; i < !normal_noise.size(); ++i)
      n_noise_param[i] = normal_noise[i];
  }

  g2o::SparseOptimizer opt;

  MatchableWorld* world = new MatchableWorld();
  world->mutableParams().resolution = resolution;
  world->mutableParams().height = world_size[0];
  world->mutableParams().width = world_size[1];
  world->mutableParams().num_points = num_points;
  world->mutableParams().num_lines = num_lines;
  world->mutableParams().num_planes = num_planes;
  world->createGrid();

  WorldSimulator ws;
  ws.mutableParams().factors_types.point_factors = has_point_point_factor;
  ws.mutableParams().factors_types.line_factors = has_line_line_factor;
  ws.mutableParams().factors_types.plane_factors = has_plane_plane_factor;
  ws.mutableParams().factors_types.line_point_factors = has_line_point_factor;
  ws.mutableParams().factors_types.plane_line_factors = has_plane_line_factor;
  ws.mutableParams().factors_types.plane_point_factors = has_plane_point_factor;
  
  ws.mutableParams().num_poses = num_poses;
  ws.mutableParams().sense_radius = sense_radius;

  ws.mutableParams().has_noise = simulator_has_noise;
  ws.mutableParams().point_noise_stats = t_noise_param;
  ws.mutableParams().normal_noise_stats = n_noise_param;
  
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
