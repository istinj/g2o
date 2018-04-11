#include "edge_se3_matchable.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/stuff/opengl_primitives.h"
#endif


namespace g2o{
  namespace matchables{

    EdgeSE3Matchable::EdgeSE3Matchable():
      BaseBinaryEdge<7,Matchable,VertexSE3Chord,VertexMatchable>(){
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

      VertexSE3Chord *v_from = static_cast<VertexSE3Chord*>(_vertices[0]);
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


    
#ifdef G2O_HAVE_OPENGL
    EdgeSE3MatchableDrawAction::EdgeSE3MatchableDrawAction(): DrawAction(typeid(EdgeSE3Matchable).name()){}

    HyperGraphElementAction* EdgeSE3MatchableDrawAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_) {
      
      if (typeid(*element).name()!=_typeName)
        return 0;
      refreshPropertyPtrs(params_);
      if (! _previousParams)
        return this;

      if (_show && !_show->value())
        return this;

      EdgeSE3Matchable* e =  static_cast<EdgeSE3Matchable*>(element);
      VertexSE3Chord* fromEdge = static_cast<VertexSE3Chord*>(e->vertex(0));
      VertexMatchable* toEdge   = static_cast<VertexMatchable*>(e->vertex(1));
      
      if (! fromEdge || ! toEdge)
        return this;
      
      Isometry3 fromTransform=fromEdge->estimate();
      glColor3f(LANDMARK_EDGE_COLOR);
      glPushAttrib(GL_ENABLE_BIT);
      glDisable(GL_LIGHTING);
      glBegin(GL_LINES);
      glVertex3f((float)fromTransform.translation().x(),(float)fromTransform.translation().y(),(float)fromTransform.translation().z());
      glVertex3f((float)toEdge->estimate().point().x(),(float)toEdge->estimate().point().y(),(float)toEdge->estimate().point().z());
      glEnd();
      glPopAttrib();
      return this;
    }
#endif
  }
}
