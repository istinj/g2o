#pragma once

#include <g2o/core/eigen_types.h>

namespace g2o{
  namespace matchables{
    class Matchable{
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

      enum Type {Point=0, Line=1, Plane=2};

      typedef Eigen::Matrix<number_t,5,1,Eigen::ColMajor> Vector5;
      typedef Eigen::Matrix<number_t,13,1,Eigen::ColMajor> Vector13;
      typedef Eigen::DiagonalMatrix<number_t,3> DiagMatrix3;

      Matchable(){}

      Matchable(Type type_,
                Vector3 point_,
                Matrix3 R_ = Matrix3::Zero());

      Matchable& operator *= (const Vector5 &v){

        Vector3 dp(v[0],v[1],v[2]);
        Matrix3 dR;
        dR = Eigen::AngleAxisd(v[3],Vector3::UnitY())*Eigen::AngleAxisd(v[4],Vector3::UnitZ());

        _point += dp;
        _R *= dR;

        return *this;
      }

      Matchable transform(const Isometry3 &T) const{
        return Matchable(_type,T*_point,T.linear()*_R);
      }

      Matchable perturb(const Vector5 &v) const{
        Vector3 dp(v[0],v[1],v[2]);
        Matrix3 dR;
        dR = Eigen::AngleAxisd(v[3],Vector3::UnitY())*Eigen::AngleAxisd(v[4],Vector3::UnitZ());

        Vector3 point = _point + dp;
        Matrix3 R  = _R * dR;

        return Matchable(_type,point,R);
      }

      Vector13 toVector() const;

      const Type &type() const{return _type;}
      const Vector3 &point() const {return _point;}
      const Matrix3 &R() const {return _R;}

      void setZero();

    private:
      Type _type;
      Vector3 _point;
      Matrix3 _R;
      DiagMatrix3 _Omega;
    };
  }
}
