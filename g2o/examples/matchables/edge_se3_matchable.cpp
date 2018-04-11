#include "edge_se3_matchable.h"

namespace g2o{
  namespace matchables{

    EdgeSE3Matchable::EdgeSE3Matchable():
      BaseBinaryEdge<7,Matchable,VertexSE3,VertexMatchable>(){
      _information.setIdentity();
    }

    bool EdgeSE3Matchable::read(std::istream &is){
      int t;
      Vector3 p;
      Matrix3 R;

      is >> t;
      is >> p[0] >> p[1] >> p[2];
      is >> R(0,0) >> R(0,1) >> R(0,2) >> R(1,0) >> R(1,1) >> R(1,2) >> R(2,0) >> R(2,1) >> R(2,2);

      Matchable::Type type;
      switch(t){
        case 0:
          type = Matchable::Type::Point;
          break;
        case 1:
          type = Matchable::Type::Line;
          break;
        case 2:
          type = Matchable::Type::Plane;
          break;
        default:
          throw std::runtime_error("[EdgeSE3Matchable][Read] irrumati!!!");
      }

      setMeasurement(Matchable(type,p,R));

      if (is.bad())
        return false;

      for ( int i=0; i<_information.rows() && is.good(); i++)
        for (int j=i; j<_information.cols() && is.good(); j++){
          is >> _information(i,j);
          if (i!=j)
            _information(j,i)=_information(i,j);
        }

      if (is.bad())
        _information.setIdentity();

      return true;
    }

    bool EdgeSE3Matchable::write(std::ostream &os) const{
      Vector3 p = _measurement.point();
      Matrix3 R = _measurement.R();

      os << _measurement.type() << " ";
      os << p[0] << " " << p[1] << " " << p[2] << " ";
      os << R(0,0) << " " << R(0,1) << " " << R(0,2) << " "
         << R(1,0) << " " << R(1,1) << " " << R(1,2) << " "
         << R(2,0) << " " << R(2,1) << " " << R(2,2) << " ";

      for (int i=0; i<_information.rows(); i++)
        for (int j=i; j<_information.cols(); j++)
          os << _information(i,j) << " ";

      return os.good();
    }

    void EdgeSE3Matchable::computeError(){

      VertexSE3 *v_from = static_cast<VertexSE3*>(_vertices[0]);
      VertexMatchable *v_to = static_cast<VertexMatchable*>(_vertices[1]);

      const Isometry3 &pose = v_from->estimate();
      const Vector3 &t = pose.translation();
      const Matrix3 &R = pose.linear();

      const Vector3 &pl = v_to->estimate().point();
      const Matrix3 &Rl = v_to->estimate().R();

      const Vector3 &pz = _measurement.point();
      const Matrix3 &Rz = _measurement.R();

      const Vector3 ep = Rl.transpose()*(R*pz + t - pl);
      const Vector3 ed = (R*Rz - Rl).col(0);
      const float eo    = (R*Rz).col(0).transpose() * Rl.col(0);

      _error.block<3,1>(0,0) = ep;
      _error.block<3,1>(3,0) = ed;
      _error[6] = eo;

    }
  }
}
