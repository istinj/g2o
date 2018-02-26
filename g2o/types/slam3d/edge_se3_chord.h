#pragma once

#include "g2o/core/base_binary_edge.h"

#include "g2o_types_slam3d_api.h"
#include "vertex_se3_chord.h"

namespace g2o {
    
  class G2O_TYPES_SLAM3D_API EdgeSE3Chord : public BaseBinaryEdge<12, Isometry3, VertexSE3Chord, VertexSE3Chord> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      
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


    virtual number_t initialEstimatePossible(const OptimizableGraph::VertexSet& /*from*/, 
                                             OptimizableGraph::Vertex* /*to*/) { 
      return 1.;
    }

    virtual void initialEstimate(const OptimizableGraph::VertexSet& /*from_*/,
                                 OptimizableGraph::Vertex* /*to*/);

  };

#ifdef G2O_HAVE_OPENGL
  //! @brief visualization
  class G2O_TYPES_SLAM3D_API EdgeSE3ChordDrawAction : public DrawAction {
  public:
    EdgeSE3ChordDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,
                                                HyperGraphElementAction::Parameters* params);
  };
#endif
  
} //ia end namespace g2o
