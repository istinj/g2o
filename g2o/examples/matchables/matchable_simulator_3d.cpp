#include <iostream>
#include <fstream>

#include "matchable_world_simulator.h"

//ia include g2o core stuff
#include "g2o/stuff/command_args.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"

using namespace g2o;
using namespace matchables;

int main(int argc, char** argv) {

  g2o::VertexSE3Chord v;

  if (argc < 2)
    throw std::runtime_error("<program> [options] filename");

  float resolution = 1.0f;
  int width=10;
  int height=10;
  int num_poses=10;
  std::string filename;

  int c=1;
  while (c<argc) {
    if(!strcmp(argv[c],"-w")){
      c++;
      width=std::atoi(argv[c]);
    } else if (!strcmp(argv[c],"-h")) {
      c++;
      height=std::atoi(argv[c]);
    } else if(!strcmp(argv[c],"-n")){
      c++;
      num_poses=std::atoi(argv[c]);
    } else if(!strcmp(argv[c],"-r")){
      c++;
      resolution=std::atof(argv[c]);
    } else {
      filename = argv[c];
    }
    c++;
  }

  g2o::SparseOptimizer opt;

  MatchableWorld *world = new MatchableWorld();
  world->setResolution(resolution);
  world->setWidth(width);
  world->setHeight(height);
  world->createGrid();
  world->removeWalls(num_poses);

  WorldSimulator ws;
  ws.setNumPoses(num_poses);
  ws.setSenseRadius(2*resolution);
  ws.setVertices(&(opt.vertices()));
  ws.setEdges(&(opt.edges()));

  ws.init();
  
  ws.setWorld(world);
  ws.compute();

  opt.save(filename.c_str());

  return 0;
}
