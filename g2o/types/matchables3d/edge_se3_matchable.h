#pragma once

#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/vertex_se3_chord.h>

#include "g2o_types_matchable_api.h"
#include "vertex_matchable.h"

namespace g2o {
  namespace matchables {
    
    class G2O_TYPES_MATCHABLE_API EdgeSE3Matchable : public BaseBinaryEdge<7, Matchable, VertexSE3Chord, VertexMatchable> {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

      //! @brief default ctor
      EdgeSE3Matchable();

      //! @brief deserialization
      virtual bool read(std::istream &is);
      //! @brief serialization
      virtual bool write(std::ostream &os) const;

      //! @brief computes the error
      void computeError();

      //! @brief computes the jacobian
      void linearizeOplus();

      inline void setMeasurement(const Matchable &m) {
        _measurement = m;
      }

      //! @brief states that the first element will set the initial estimate of the second element
      number_t initialEstimatePossible(const OptimizableGraph::VertexSet& /*from*/,
                                       OptimizableGraph::Vertex* to) override {

        VertexMatchable* v_to = dynamic_cast<VertexMatchable*>(to);

        if (!v_to) {
          return -1.0;
        }

        if (v_to->estimate().type() == _measurement.type())
          return 1.0; //ia is this good? this will set the matchable from the pose.
        else
          return -1.0; //should be negative if it is not possible to compute the initial guess
      }

      //! @brief actually computes the initial guess
      virtual void initialEstimate(const OptimizableGraph::VertexSet& /*from*/,
                                   OptimizableGraph::Vertex* /*to*/);
    };


#ifdef G2O_HAVE_OPENGL
    //! @brief visualization stuff
    class EdgeSE3MatchableDrawAction: public DrawAction {
    public:
      EdgeSE3MatchableDrawAction();
      virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,
                                                  HyperGraphElementAction::Parameters* params_);
    };
#endif
  }
}
