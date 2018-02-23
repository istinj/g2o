#pragma once

#include "g2o/core/base_binary_edge.h"
#include "vertex_se3_chord.h"

namespace g2o {
  namespace chordal {
    
    class EdgeSE3Chord : public BaseBinaryEdge<12, Isometry3, VertexSE3Chord, VertexSE3Chord> {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      //! @brief ctor
      EdgeSE3Chord();

      //! @brief read/write in a stream
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      //! @brief computes the error according to the approximation
      //!        of the chordal distance between the two quantities
      void computeError();
      
      //! @brief computes the right jacobians
      void linearizeOplus();

      void setMeasurement(const Isometry3& meas) {
        _measurement = meas;
      }


      virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& from, 
          OptimizableGraph::Vertex* to) { 
        return 1.;
      }

      virtual void initialEstimate(const OptimizableGraph::VertexSet& from_, OptimizableGraph::Vertex* to);

    };

    class EdgeSE3ChordDrawAction : public DrawAction {
    public:
      EdgeSE3ChordDrawAction();
      virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,
                                                  HyperGraphElementAction::Parameters* params);
    };

  } //ia end namespace chordal
} //ia end namespace g2o
