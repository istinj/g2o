//ia simulator class. it owns nothing. you create a world outside
//ia and you pass it togheter with empty edges and vertices.
//ia it will shit out vertices and measurement based on the factors you set
//ia it performs the actions MOVE (moves inside the world) and SENSE (senses the things
//ia and creates an odometry)
//ia EXTRA encodes gaussian noise in the measurement (TODO)

#pragma once
#include "matchable_world.h"

namespace g2o {
  namespace matchables {
    
    class WorldSimulator {
    public:

      struct MatchableSimulatorFactors {
        MatchableSimulatorFactors() {
          point_factors = false;
          line_factors = false;
          plane_factors = false;
        }
        bool point_factors;
        bool line_factors;
        bool plane_factors;
      };
      
      WorldSimulator();
      virtual ~WorldSimulator();

      inline const number_t& senseRadius() const {return _sense_radius;}
      inline void setSenseRadius(const number_t& sense_radius_) {_sense_radius = sense_radius_;}

      //ia non const return in order to set 
      inline MatchableSimulatorFactors& factorTypes() {return _factors_types;}

      inline void setVertices(HyperGraph::VertexIDMap* vertices_) {_vertices = vertices_;}
      inline HyperGraph::VertexIDMap* vertices() const {return _vertices;}
      
      inline void setEdges(HyperGraph::EdgeSet* edges_) {_edges = edges_;}
      inline HyperGraph::EdgeSet* edges() const {return _edges;}

      inline void setWorld(MatchableWorld* world_) {_world = world_;}
      inline MatchableWorld* world() const {return _world;}

      //ia checks that everything is ok
      void init();

    protected:
      //ia vertices and edges to be inzepped (not owned)
      HyperGraph::VertexIDMap* _vertices = 0;
      HyperGraph::EdgeSet*     _edges = 0;
      MatchableWorld*          _world = 0;

      //ia parameters
      number_t _sense_radius;
      int _num_poses;
      MatchableSimulatorFactors _factors_types;
    };
    
  }
}
