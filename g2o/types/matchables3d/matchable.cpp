#include "matchable.h"

namespace g2o {
  namespace matchables {
    const number_t Matchable::_epsilon = 1e-6;
    
    Matchable::Matchable(const Type& type_,
                         const Vector3& point_,
                         const Matrix3& R_):
      _type(type_),
      _point(point_),
      _rotation(R_){

      _omega.setZero();
      switch(type_){
        case Point:
          _omega.setIdentity();
          break;
        case Line:
          _omega.diagonal()[0]=_epsilon;
          _omega.diagonal()[1]=1;
          _omega.diagonal()[2]=1;
          break;
        case Plane:
          _omega.diagonal()[0]=1;
          _omega.diagonal()[1]=_epsilon;
          _omega.diagonal()[2]=_epsilon;
          break;
        default: break;
      }
    }

    Vector13 Matchable::toVector() const {
      Vector13 ret;
      ret << _type,
          _point.x(),_point.y(),_point.z(),
          _rotation(0,0),_rotation(0,1),_rotation(0,2),
          _rotation(1,0),_rotation(1,1),_rotation(1,2),
          _rotation(2,0),_rotation(2,1),_rotation(2,2);
      return ret;
    }

    void Matchable::applyMinimalPertInPlace(const Vector5& dm_) {
      Matrix3 dR;
      dR = AngleAxis(dm_[3],Vector3::UnitY())*AngleAxis(dm_[4],Vector3::UnitZ());

      _point += dm_.head(3);
      _rotation = _rotation * dR;

      //ia enforcing orthonormality
      const Matrix3 R_backup = _rotation;
      Matrix3 E = R_backup.transpose() * R_backup;
      E.diagonal().array() -= 1;

      _rotation -= 0.5*R_backup*E;
    }

    Matchable Matchable::applyMinimalPert(const Vector5& dm_) const  {
      Matrix3 dR;
      dR = AngleAxis(dm_[3],Vector3::UnitY()) * AngleAxis(dm_[4],Vector3::UnitZ());

      Vector3 point = _point + dm_.head(3);
      Matrix3 R  = _rotation * dR;

      //ia enforcing orthonormality
      const Matrix3 R_backup = R;
      Matrix3 E = R_backup.transpose() * R_backup;
      E.diagonal().array() -= 1;

      R -= 0.5*R_backup*E;

      return Matchable(_type,point,R);
    }

    void Matchable::computeRotationMatrixZXY(const Vector3& normal_) {
      number_t d = std::sqrt(normal_.x()*normal_.x() + normal_.y()*normal_.y());

      const number_t& dirx = normal_.x();
      const number_t& diry = normal_.y();
      const number_t& dirz = normal_.z();

      if(d > std::numeric_limits<number_t>::min()) {
        _rotation <<
          dirx, diry/d,  dirx*dirz/d,
          diry, -dirx/d, diry*dirz/d,
          dirz, 0,       -d;
      } else {
        _rotation << 0,1,0,
          0,0,1,
          1,0,0;
      }
    }
    
  }
}
