#include "vertex_se3_chord.h"
#include "g2o/core/factory.h"
#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/stuff/opengl_primitives.h"
#endif

#include <iostream>
#include "g2o/core/cache.h"

using namespace Eigen;

namespace g2o {

  VertexSE3Chord::VertexSE3Chord() :
    BaseVertex<6, Isometry3>(),
    _numOplusCalls(0)
  {
    setToOriginImpl();
    updateCache();
  }

  bool VertexSE3Chord::read(std::istream& is)
  {
    Vector7 est;
    for (int i=0; i<7; i++)
      is  >> est[i];
    setEstimate(internal::fromVectorQT(est));
    return true;
  }

  bool VertexSE3Chord::write(std::ostream& os) const
  {
    Vector7 est=internal::toVectorQT(_estimate);
    for (int i=0; i<7; i++)
      os << est[i] << " ";
    return os.good();
  }

  VertexSE3ChordWriteGnuplotAction::VertexSE3ChordWriteGnuplotAction(): WriteGnuplotAction(typeid(VertexSE3Chord).name()){}

  HyperGraphElementAction* VertexSE3ChordWriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;
    WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
    if (!params->os){
      std::cerr << __PRETTY_FUNCTION__ << ": warning, no valid os specified" << std::endl;
      return 0;
    }
    
    VertexSE3Chord* v =  static_cast<VertexSE3Chord*>(element);
    Vector6 est=internal::toVectorMQT(v->estimate());
    for (int i=0; i<6; i++)
      *(params->os) << est[i] << " ";
    *(params->os) << std::endl;
    return this;
  }

#ifdef G2O_HAVE_OPENGL
  VertexSE3ChordDrawAction::VertexSE3ChordDrawAction(): DrawAction(typeid(VertexSE3Chord).name()){
    _cacheDrawActions = 0;
  }

  bool VertexSE3ChordDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
    if (!DrawAction::refreshPropertyPtrs(params_))
      return false;
    if (_previousParams){
      _triangleX = _previousParams->makeProperty<FloatProperty>(_typeName + "::TRIANGLE_X", .2f);
      _triangleY = _previousParams->makeProperty<FloatProperty>(_typeName + "::TRIANGLE_Y", .05f);
    } else {
      _triangleX = 0;
      _triangleY = 0;
    }
    return true;
  }

  HyperGraphElementAction* VertexSE3ChordDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
                 HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;
    initializeDrawActionsCache();
    refreshPropertyPtrs(params_);

    if (! _previousParams)
      return this;
    
    if (_show && !_show->value())
      return this;

    VertexSE3Chord* that = static_cast<VertexSE3Chord*>(element);

    glColor3f(POSE_VERTEX_COLOR);
    glPushMatrix();
    glMultMatrixd(that->estimate().matrix().cast<double>().eval().data());
    opengl::drawArrow2D(_triangleX->value(), _triangleY->value(), _triangleX->value()*.3f);
    drawCache(that->cacheContainer(), params_);
    drawUserData(that->userData(), params_);
    glPopMatrix();
    return this;
  }
#endif

}
