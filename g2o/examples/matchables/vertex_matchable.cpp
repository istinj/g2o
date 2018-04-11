#include "vertex_matchable.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_primitives.h"
#include "g2o/stuff/opengl_wrapper.h"
#endif

#include <typeinfo>


namespace g2o{
  namespace matchables{

    VertexMatchable::VertexMatchable() :
      BaseVertex<5,Matchable>() {
    }

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



    VertexMatchableDrawAction::VertexMatchableDrawAction(): DrawAction(typeid(VertexMatchable).name()){
      _cacheDrawActions = 0;
    }

    bool VertexMatchableDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
      if (! DrawAction::refreshPropertyPtrs(params_))
        return false;
      if (_previousParams){
        _pointSize = _previousParams->makeProperty<FloatProperty>(_typeName + "::POINT_SIZE", 1.);
      } else {
        _pointSize = 0;
      }
      return true;
    }
    
    HyperGraphElementAction* VertexMatchableDrawAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params) {

    if (typeid(*element).name()!=_typeName)
      return 0;
    initializeDrawActionsCache();
    refreshPropertyPtrs(params);
    if (! _previousParams)
      return this;
    
    if (_show && !_show->value())
      return this;
    VertexMatchable* that = static_cast<VertexMatchable*>(element);
    
    glPushMatrix();
    glPushAttrib(GL_ENABLE_BIT | GL_POINT_BIT);
    glDisable(GL_LIGHTING);
    
    switch (that->estimate().type()) {
    case Matchable::Type::Point:
      glColor3f(LANDMARK_VERTEX_COLOR);
      break;
    case Matchable::Type::Line:
      glColor3f(0.1,0.7,0.1);
      break;
    case Matchable::Type::Plane:
      glColor3f(0.7,0.7,0.1);
      break;
    default:
      throw std::runtime_error("[VertexMatchableDrawAction] unsupported matchable type");
    }

    const Vector3& point = that->estimate().point();
    
    float ps = _pointSize ? _pointSize->value() :  1.f;
    glTranslatef((float)point(0),(float)point(1),(float)point(2));
    opengl::drawPoint(ps);
    glPopAttrib();
    drawCache(that->cacheContainer(), params);
    drawUserData(that->userData(), params);
    glPopMatrix();
    
    return this;
    }    

  }
}
