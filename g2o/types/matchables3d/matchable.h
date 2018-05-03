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

      Matchable(const Type& type_,
                const Vector3& point_,
                const Matrix3& R_ = Matrix3::Identity());

      inline const Type& type() const{return _type;}
      inline const Vector3& point() const {return _point;}
      inline const Matrix3& rotation() const {return _rotation;}
      inline const DiagMatrix3& omega() const {return _omega;}
      
      inline void setZero() {
        _point.setZero();
        _rotation.setIdentity();
      }

      inline Matchable applyTransform(const Isometry3& T) const {
        Matchable m(_type,T*_point);

        if(_type != Type::Point)
          m.setRotation(T.linear()*_rotation);

        return m;
      }
      
      inline void applyTransformInPlace(const Isometry3& T) {
        _point = T*_point;
        if(_type != Type::Point)
          setRotation(T.linear()*_rotation);
      }
      
      Matchable applyMinimalPert(const Vector5& v) const;

      void applyMinimalPertInPlace(const Vector5& v);

      void computeRotationMatrixZXY(const Vector3& normal_);

      void setRotation(const Matrix3& rotation_){_rotation = rotation_;}

      Vector13 toVector() const;


    protected:
      Type    _type;
      Vector3 _point;
      Matrix3 _rotation;
      DiagMatrix3 _omega;

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
