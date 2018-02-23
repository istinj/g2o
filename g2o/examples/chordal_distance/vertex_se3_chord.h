#pragma once

#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "se3_chordal_ops.h"

namespace g2o {
  namespace chordal {
    
    class VertexSE3Chord : public BaseVertex<6, Isometry3> {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      VertexSE3Chord();
      
      //! @brief sets the estimate to the origin
      virtual void setToOriginImpl() {
        _estimate = Isometry3::Identity();
      }

      //! @brief read and write to a stream
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      //! @brief update the estimate of the vertex.
      //! @param[in] update: array containing (x,y,z,aplha,beta,gamma)
      //!                    <x,y,z> -> translational update
      //!                    <alpha,beta,gamma> -> rot update (Euler Angles)
      virtual void oplusImpl(const double* update) {
        Eigen::Map<const Vector6> v(update);
        Isometry3 new_estimate = v2t(v) * _estimate;

        //ia reinforcing the rotation constraint
        const Matrix3 rotation = new_estimate.linear();
        Matrix3 rotation_squared = rotation.transpose() * rotation;
        rotation_squared.diagonal().array() -= 1;
        new_estimate.linear() -= 0.5*rotation*rotation_squared;

        _estimate = new_estimate;
      }
    };

    class VertexSE3ChordDrawAction : public DrawAction {
    public:
      VertexSE3ChordDrawAction();
      virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,
                                                  HyperGraphElementAction::Parameters* params);
    protected:
      virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params);
      FloatProperty* _triangleX;
      FloatProperty* _triangleY;
    };

  } //ia end namespace chordal
} //ia end namespace g2o