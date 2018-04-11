#include "matchable.h"

namespace g2o {
  namespace matchables{
    Matchable::Matchable(Type type_,
                         Vector3 point_,
                         Matrix3 R_):
      _type(type_),
      _point(point_),
      _R(R_){

      _Omega.setZero();
      switch(type_){
        case Point:
          _Omega.setIdentity();
        case Line:
          _Omega.diagonal()[1]=1;
          _Omega.diagonal()[2]=1;
          break;
        case Plane:
          _Omega.diagonal()[0]=1;
          break;
        default: break;
      }
    }

    void Matchable::setZero(){
      _point.setZero();
    }
  }
}
