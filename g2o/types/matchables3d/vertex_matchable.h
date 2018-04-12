#pragma once

#include <g2o/core/base_vertex.h>
#include <g2o/core/eigen_types.h>
#include <g2o/core/hyper_graph_action.h>

#include "g2o_types_matchable_api.h"
#include "matchable.h"

namespace g2o {
  namespace matchables {
    
    class G2O_TYPES_MATCHABLE_API VertexMatchable : public BaseVertex<5, Matchable> {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

      //! @brief deafault ctor
      VertexMatchable();

      //! @brief sets matchable point component to 0
      virtual void setToOriginImpl(){
        _estimate.setZero();
      }

      virtual bool read(std::istream &is);
      virtual bool write(std::ostream &os) const;

      virtual void oplusImpl(const double *update){
        Vector5 v;
        v << update[0],update[1],update[2],update[3],update[4];

        _estimate *= v;
      }
    };


    #ifdef G2O_HAVE_OPENGL
    class G2O_TYPES_MATCHABLE_API VertexMatchableDrawAction: public DrawAction {
    public:
      VertexMatchableDrawAction();
      virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_);

    protected:
      FloatProperty *_pointSize;
      virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
    };
    #endif
  }
}
