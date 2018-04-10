#pragma once

#include <g2o/core/base_vertex.h>
#include <g2o/core/eigen_types.h>
#include "matchable.h"

namespace g2o{
  namespace matchables{    
    class VertexMatchable : public BaseVertex<5,Matchable>{
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

      VertexMatchable(Matchable::Type type_, const Vector3F &point_, Matrix3F R_ = Matrix3F::Zero());

      virtual void setToOriginImpl(){
        _estimate.setZero();
      }

      virtual void oplusImpl(const double *update){
        Matchable::Vector5F v;
        v << update[0],update[1],update[2],update[3],update[4];

        _estimate *= v;
      }

      virtual bool read(std::istream &is);
      virtual bool write(std::ostream &os) const;

    protected:
      Matchable::Type _type;

    };
  }
}
