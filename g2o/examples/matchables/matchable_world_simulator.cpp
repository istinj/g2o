#include "matchable_world_simulator.h"

namespace g2o {
  namespace matchables {
    WorldSimulator::WorldSimulator() {}

    WorldSimulator::~WorldSimulator() {}

    void WorldSimulator::init() {
      if (!_vertices) {
        throw std::runtime_error("please set the vertices");
      }

      if (!_edges) {
        throw std::runtime_error("please set the edges");
      }
      
    }
  }
}
