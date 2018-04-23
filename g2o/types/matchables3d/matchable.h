#pragma once
#include <set>
#include <vector>
#include <memory>
#include <g2o/core/eigen_types.h>

namespace g2o{
  namespace matchables{
    
    typedef Eigen::DiagonalMatrix<number_t,3> DiagMatrix3;
    
    class Matchable{
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

      enum Type {Point=0, Line=1, Plane=2};

      Matchable(){}

      Matchable(Type type_,
                Vector3 point_,
                Matrix3 R_ = Matrix3::Zero());

      Matchable& operator *= (const Vector5 &v) {

        Vector3 dp(v[0],v[1],v[2]);
        Matrix3 dR;
        dR = AngleAxis(v[3],Vector3::UnitY())*AngleAxis(v[4],Vector3::UnitZ());

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
        dR = AngleAxis(v[3],Vector3::UnitY())*AngleAxis(v[4],Vector3::UnitZ());

        Vector3 point = _point + dp;
        Matrix3 R  = _R * dR;

        return Matchable(_type,point,R);
      }

      Vector13 toVector() const;

      inline const Type& type() const{return _type;}
      inline const Vector3& point() const {return _point;}
      inline const Matrix3& R() const {return _R;}
      inline const DiagMatrix3& omega() const {return _Omega;}

      void setZero();

    protected:
      Type    _type;
      Vector3 _point;
      Matrix3 _R;
      DiagMatrix3 _Omega;

    public:
      static const number_t _epsilon;
    };

    typedef std::vector<Matchable> MatchableVector;
    typedef std::set<Matchable> MatchableSet;
    typedef std::pair<Matchable, Matrix7> MatchableMatrix7Pair;
    typedef std::vector<MatchableMatrix7Pair> MatchableMatrix7PairVector;
    typedef std::shared_ptr<Matchable> MatchablePtr;
    typedef std::set<MatchablePtr> MatchablePtrSet;

  }
}
