#pragma once

#include <vector>
#include <set>
#include <map>
#include <Eigen/Core>
#include "g2o/core/sparse_optimizer.h"

namespace g2o {


// @brief sort of local map
class LocalStar {
public:
  LocalStar();
  virtual ~LocalStar();

  inline const HyperGraph::VertexIDMap& vertices() {return _vertices;}
  inline void setMaxSize(const int max_size_) {_max_size = max_size_;}
  inline const int maxSize() const {return _max_size;}
  inline void setLevel(const int level_) {_level = level_;}
  inline const int level() const {return _level;}

  inline void setGauge(const int vertex_id_) {
    _gauge = _vertices[vertex_id_];
  }
  inline HyperGraph::Vertex* gauge() const {return _gauge;}
  inline const bool isFull() const {return _vertices.size() == _max_size;}


  inline void addVertex(HyperGraph::Vertex* v_) {
    _vertices.insert(std::make_pair(v_->id(), v_));
  }

protected:

  int _max_size;
  int _level;
  int _gauge_dimension;

  HyperGraph::EdgeSet _total_edges;
  HyperGraph::EdgeSet _frontier_edges;
  HyperGraph::VertexIDMap _vertices;
  HyperGraph::Vertex* _gauge;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class StarHandler {
public:
  StarHandler();
  virtual ~StarHandler();

  inline void setDistanceThresh(const double& thresh_) {_distance_thresh = thresh_;}
  inline const double& distanceThresh() const {return _distance_thresh;}
  inline const int size() const {return _stars_number;}

  void init();

  LocalStar* addStar();
  void addVertexToStar(HyperGraph::Vertex* v);
protected:
  int _stars_number;
  double _distance_thresh;
  std::vector<LocalStar*> _stars;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} //ia end namespace g2o

