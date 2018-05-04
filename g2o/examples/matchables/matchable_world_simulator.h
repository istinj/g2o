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
          point_factors = true;
          line_factors = true;
          plane_factors = true;

          //ia mixed convenction: from_to
          line_point_factor = true;
          plane_line_factor = true;
          plane_point_factor = true;
        }
        bool point_factors;
        bool line_factors;
        bool plane_factors;
        bool line_point_factor;
        bool plane_line_factor;
        bool plane_point_factor;
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

      inline void setNumPoses(const int num_poses_) {_num_poses = num_poses_;}

      //ia checks that everything is ok
      void init();

      void compute();

    protected:
      //ia vertices and edges to be inzepped (not owned)
      HyperGraph::VertexIDMap* _vertices = 0;
      HyperGraph::EdgeSet*     _edges = 0;
      MatchableWorld*          _world = 0;

      uint64_t _vertex_id;

      //ia parameters
      number_t _sense_radius;
      int _num_poses;
      MatchableSimulatorFactors _factors_types;

    private:
      void senseMatchables(g2o::VertexSE3Chord* v_);
      g2o::HyperGraph::Edge* _computePointEdge(g2o::VertexSE3Chord* vfrom_,
                                               g2o::matchables::VertexMatchable* vto_);
      g2o::HyperGraph::Edge* _computeLineEdge(g2o::VertexSE3Chord* vfrom_,
                                              g2o::matchables::VertexMatchable* vto_);
      g2o::HyperGraph::Edge* _computePlaneEdge(g2o::VertexSE3Chord* vfrom_,
                                               g2o::matchables::VertexMatchable* vto_);
      g2o::HyperGraph::Edge* _computeLinePointEdge(g2o::VertexSE3Chord* vfrom_,
                                                   g2o::matchables::VertexMatchable* vto_);
      g2o::HyperGraph::Edge* _computePlaneLineEdge(g2o::VertexSE3Chord* vfrom_,
                                                   g2o::matchables::VertexMatchable* vto_);
      g2o::HyperGraph::Edge* _computePlanePointEdge(g2o::VertexSE3Chord* vfrom_,
                                                    g2o::matchables::VertexMatchable* vto_);

      size_t point_point;
      size_t line_line;
      size_t plane_plane;
      size_t line_point;
      size_t plane_point;
      size_t plane_line;
    };
    
  }
}
