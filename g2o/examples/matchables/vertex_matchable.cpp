#include "vertex_matchable.h"

namespace g2o{
  namespace matchables{

    bool VertexMatchable::read(std::istream &is){
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
          throw std::runtime_error("[VertexMatchable][Read] irrumati!!!");
      }

      _estimate = Matchable(type,p,R);


      return true;
    }

    bool VertexMatchable::write(std::ostream &os) const{
      Vector3 p = _estimate.point();
      Matrix3 R = _estimate.R();

      os << _estimate.type() << " ";
      os << p[0] << " " << p[1] << " " << p[2] << " ";
      os << R(0,0) << " " << R(0,1) << " " << R(0,2) << " "
         << R(1,0) << " " << R(1,1) << " " << R(1,2) << " "
         << R(2,0) << " " << R(2,1) << " " << R(2,2) << " ";

      return os.good();
    }

  }
}
