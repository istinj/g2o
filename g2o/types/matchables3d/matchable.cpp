#include "matchable.h"

namespace g2o {
  namespace matchables {
    const number_t Matchable::_epsilon = 1e-6;
    
    Matchable::Matchable(const Type& type_,
                         const Vector3& point_,
                         const Matrix3& R_):
      _type(type_),
      _point(point_),
      _R(R_){

      _Omega.setZero();
      switch(type_){
        case Point:
          _Omega.setIdentity();
        case Line:
          _Omega.diagonal()[0]=_epsilon;
          _Omega.diagonal()[1]=1;
          _Omega.diagonal()[2]=1;
          break;
        case Plane:
          _Omega.diagonal()[0]=1;
          _Omega.diagonal()[1]=_epsilon;
          _Omega.diagonal()[2]=_epsilon;
          break;
        default: break;
      }
    }

    void Matchable::setZero(){
      _point.setZero();
    }

    Vector13 Matchable::toVector() const {
      Vector13 ret;
      ret << _type,
             _point.x(),_point.y(),_point.z(),
             _R(0,0),_R(0,1),_R(0,2),_R(1,0),_R(1,1),_R(1,2),_R(2,0),_R(2,1),_R(2,2);
      return ret;
    }
  }
}
