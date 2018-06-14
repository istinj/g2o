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

    //!TODO
    //1 removeWalls may be performed here (has more sense)
    //2 PLANE TO LINE EDGES
    //3 viewer application?
    //4 add noise components to the edges
    //5 initialGuess to the edges
    
    class WorldSimulator {
    public:

      typedef std::vector<g2o::HyperGraph::Edge*,
        Eigen::aligned_allocator<g2o::HyperGraph::Edge*> > EdgeVector;
      
      struct MatchableSimulatorFactors {        
        bool point_factors;
        bool line_factors;
        bool plane_factors;
        bool line_point_factors;
        bool plane_line_factors;
        bool plane_point_factors;
        
        MatchableSimulatorFactors() {
          point_factors = false;
          line_factors = false;
          plane_factors = false;

          //ia mixed convenction: from_to
          line_point_factors = false;
          plane_line_factors = false;
          plane_point_factors = false;
        }

        void clearFactors() {
          point_factors = false;
          line_factors = false;
          plane_factors = false;

          //ia mixed convenction: from_to
          line_point_factors = false;
          plane_line_factors = false;
          plane_point_factors = false;
        }

        bool checkFactors() {
          if (point_factors || line_factors || plane_factors ||
              line_point_factors || plane_line_factors || plane_point_factors)
            return true;
          return false;
        }
      };

      struct MatchableSimulatorStats {
        size_t point_point;
        size_t line_line;
        size_t plane_plane;
        size_t line_point;
        size_t plane_point;
        size_t plane_line;

        MatchableSimulatorStats() {
          point_point = 0;
          line_line   = 0;
          plane_plane = 0;
          line_point  = 0;
          plane_point = 0;
          plane_line  = 0;
        }

        void setZero() {
          point_point = 0;
          line_line   = 0;
          plane_plane = 0;
          line_point  = 0;
          plane_point = 0;
          plane_line  = 0;
        }

        void print() {
          std::cerr << "NUM PT-PT \t" << point_point << std::endl;
          std::cerr << "NUM LN-LN \t" << line_line << std::endl;
          std::cerr << "NUM PL-PL \t" << plane_plane << std::endl;
          std::cerr << "NUM LN-PT \t" << line_point << std::endl;
          std::cerr << "NUM PL-PT \t" << plane_point << std::endl;
          std::cerr << "NUM PL-LN \t" << plane_line << std::endl;
        }
      };

      struct Parameters {
        size_t   num_poses;
        number_t sense_radius;
        
        MatchableSimulatorFactors factors_types;
        MatchableSimulatorStats   simulator_stats;
        Parameters() {
          num_poses    = 0;
          sense_radius = 2.0;

          factors_types.clearFactors();
          simulator_stats.setZero();
        }
      };
      
      WorldSimulator();
      virtual ~WorldSimulator();

      //ia inline set get
      inline void setVertices(HyperGraph::VertexIDMap* vertices_) {_vertices = vertices_;}
      inline HyperGraph::VertexIDMap* vertices() const {return _vertices;}
      
      inline void setEdges(HyperGraph::EdgeSet* edges_) {_edges = edges_;}
      inline HyperGraph::EdgeSet* edges() const {return _edges;}

      inline void setWorld(MatchableWorld* world_) {_world = world_;}
      inline MatchableWorld* world() const {return _world;}

      inline const Parameters& params() const {return _params;}
      inline Parameters& mutableParams() {return _params;}
      
      //ia checks that everything is ok
      void init();

      void compute();

    protected:
      //ia sense part
      void _senseMatchables(g2o::VertexSE3Chord* v_);
      //ia moves the robot from prev_vertex, generates a new vertexSE3 and an edge between the two
      VertexSE3Chord* _moveRobot(VertexSE3Chord* from_vertex_);
      g2o::HyperGraph::Edge* _computePointEdge(g2o::VertexSE3Chord* vfrom_,
                                               g2o::matchables::VertexMatchable* vto_);
      g2o::HyperGraph::Edge* _computeLineEdge(g2o::VertexSE3Chord* vfrom_,
                                              g2o::matchables::VertexMatchable* vto_);
      g2o::HyperGraph::Edge* _computePlaneEdge(g2o::VertexSE3Chord* vfrom_,
                                               g2o::matchables::VertexMatchable* vto_);
      g2o::HyperGraph::Edge* _computeLinePointEdge(g2o::VertexSE3Chord* vfrom_,
                                                   g2o::matchables::VertexMatchable* vto_);
      g2o::HyperGraph::Edge* _computePlanePointEdge(g2o::VertexSE3Chord* vfrom_,
                                                    g2o::matchables::VertexMatchable* vto_);
      EdgeVector _computePlaneLineEdge(g2o::VertexSE3Chord* vfrom_,
                                       g2o::matchables::VertexMatchable* vto_);

      
      //ia vertices and edges to be inzepped (not owned)
      HyperGraph::VertexIDMap* _vertices = 0;
      HyperGraph::EdgeSet*     _edges = 0;
      MatchableWorld*          _world = 0;

      uint64_t _vertex_id;

      //ia parameters
      Parameters _params;
    };
    
  }
}
