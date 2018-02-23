#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace g2o {
  namespace chordal {

    //ia useful typedefs
    typedef Eigen::Matrix<double, 3, 3>     Matrix3;
    typedef Eigen::Matrix<double, 6, 6>     Matrix6;
    typedef Eigen::Matrix<double, 12, 12>   Matrix12;

    typedef Eigen::Matrix<double, 3, 1>     Vector3;
    typedef Eigen::Matrix<double, 6, 1>     Vector6;
    typedef Eigen::Matrix<double, 9, 1>     Vector9;
    typedef Eigen::Matrix<double, 12, 1>    Vector12;

    typedef Eigen::Transform<double,3,Eigen::Isometry>  Isometry3;
    
    class SE3Chord {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      SE3Chord(){
        _isometry = Isometry3::Identity();
      };

      virtual ~SE3Chord() {};

      inline const Isometry3& isometry() const {
        return _isometry;
      }

      inline Isometry3& isometry() {
        return _isometry;
      }

    protected:
      Isometry3 _isometry;
    };

  } //ia end namespace chordal
} //ia end namespace g2o