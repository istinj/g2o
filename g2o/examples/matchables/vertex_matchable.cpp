#include "vertex_matchable.h"

namespace g2o{
  namespace matchables{

    VertexMatchable::VertexMatchable(Matchable::Type type_,
                                     const Vector3F & point_,
                                     Matrix3F R_): BaseVertex<5,Matchable>() {

    }

    bool VertexMatchable::read(std::istream &is){
      int t;
      Vector3F p;
      Matrix3F R;

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
          throw std::runtime_error("[VertexMatchable][Read] irrumati!!!");
      }

      _estimate = Matchable(type,p,R);
      _type = type;

      return true;
    }

    bool VertexMatchable::write(std::ostream &os) const{
      Vector3F p = _estimate.point();
      Matrix3F R = _estimate.R();

      os << _type << " ";
      os << p[0] << " " << p[1] << " " << p[2] << " ";
      os << R(0,0) << " " << R(0,1) << " " << R(0,2) << " "
         << R(1,0) << " " << R(1,1) << " " << R(1,2) << " "
         << R(2,0) << " " << R(2,1) << " " << R(2,2) << " ";

      return os.good();
    }

  }
}
