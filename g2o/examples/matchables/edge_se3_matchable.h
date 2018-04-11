#pragma once

#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include "vertex_matchable.h"

namespace g2o{
  namespace matchables{
    class EdgeSE3Matchable : public BaseBinaryEdge<7,Matchable,VertexSE3,VertexMatchable> {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

      EdgeSE3Matchable();

      void computeError();

      virtual bool read(std::istream &is);

      virtual bool write(std::ostream &os) const;

      inline void setMeasurement(const Matchable &m){
        _measurement = m;
      }
    };
  }
}
