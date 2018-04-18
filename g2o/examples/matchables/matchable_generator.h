#include <iostream>

//ia include types
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"
#include "g2o/types/matchables3d/vertex_matchable.h"
#include "g2o/types/matchables3d/edge_se3_matchable.h"

//ia include g2o core stuff
#include "g2o/stuff/sampler.h"
#include "g2o/stuff/command_args.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/factory.h"

namespace g2o {
  namespace matchables {
    
    class MatchableGenerator {
    public:

      //ia parameters srrg2 style
      struct Parameters {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Parameters() {
          num_poses  = 0;
          num_points = 0;
          num_lines  = 0;
          num_planes = 0;
          world_size = 0.0;

          has_point_factors = false;
          has_line_factors  = false;
          has_plane_factors = false;

          noise_vertices = false;
          noise_edges    = false;
        }        
        int num_poses;
        int num_points;
        int num_lines;
        int num_planes;
        float world_size;

        //ia just homogenous factors for now
        bool has_point_factors;
        bool has_line_factors;
        bool has_plane_factors;

        //ia noise
        bool noise_vertices;
        bool noise_edges;
      };
      
      MatchableGenerator();
      virtual ~MatchableGenerator();

      //! @brief inline set/get
      inline void setVertices(HyperGraph::VertexIDMap* vertices_) {_vertices = vertices_;}
      inline HyperGraph::VertexIDMap* vertices() {return _vertices;}

      inline void setEdges(HyperGraph::EdgeSet* edges_) {_edges = edges_;}
      inline HyperGraph::EdgeSet* edges() {return _edges;}

      inline Parameters& config() {_is_setup = false; return _params;}
      inline const Parameters& constConfig() const {return _params;}

      //! @brief checks the consistency of the parameters
      //! @brief it must be called just after you finish to set the parameters
      void setup();
      
      //! @brief initializes the things -> called just before the black magic happens
      void init();

      //! @brief generates vertices and edges according to the parameters
      void compute();

    protected:
      //! @brief computes the poses of the robot
      void _computePoses();
      //! @brief those compute the landmarks 
      void _computePoints();
      void _computeLines();
      void _computePlanes();
      //! @brief those compute the edges
      void _computeEdges();
      HyperGraph::Edge* _computePointEdge(VertexSE3Chord* vfrom_,
                                          VertexMatchable* vto_);
      HyperGraph::Edge* _computeLineEdge(VertexSE3Chord* vfrom_,
                                         VertexMatchable* vto_);
      HyperGraph::Edge* _computePlaneEdge(VertexSE3Chord* vfrom_,
                                          VertexMatchable* vto_);

      Factory*                 _factory   = nullptr;
      HyperGraph::VertexIDMap* _vertices  = nullptr;
      HyperGraph::EdgeSet*     _edges     = nullptr;

      Parameters  _params;
      bool        _is_setup;
      bool        _is_init;

      int _id;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    
  } //ia end namespace matchables
} //ia end namespace g2o
