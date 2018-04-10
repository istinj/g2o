#pragma once

#include <g2o/core/eigen_types.h>

namespace g2o{
  namespace matchables{
    class Matchable{
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

      enum Type {Point=0, Line=1, Plane=2};

      typedef Eigen::Matrix<float,5,1,Eigen::ColMajor> Vector5F;
      typedef Eigen::Matrix<float,13,1,Eigen::ColMajor> Vector13F;
      typedef Eigen::DiagonalMatrix<float,3> DiagMatrix3F;

      Matchable();

      Matchable(Type type_,
                Vector3F point_,
                Matrix3F R_ = Matrix3F::Zero());

      Matchable& operator *= (const Vector5F &v){

        Vector3F dp(v[0],v[1],v[2]);
        Matrix3F dR;
        dR = Eigen::AngleAxisf(v[3],Vector3F::UnitY())*Eigen::AngleAxisf(v[4],Vector3F::UnitZ());

        _point += dp;
        _R *= dR;

        return *this;
      }

      void fromVector(const Vector13F &v);

      Vector13F toVector() const;


      const Vector3F &point() const {return _point;}
      const Matrix3F &R() const {return _R;}

      void setZero();

    private:
      Vector3F _point;
      Matrix3F _R;
      DiagMatrix3F _Omega;
    };
  }
}
