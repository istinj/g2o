#include "vertex_se3_chord.h"

//ia other stuff
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/stuff/opengl_primitives.h"

namespace g2o {
  namespace chordal {
    
    VertexSE3Chord::VertexSE3Chord() : 
        BaseVertex<6, Isometry3>() {
      setToOriginImpl();
    }

    bool VertexSE3Chord::read(std::istream& is) {
      Vector7 est = Vector7::Zero();
      for (uint8_t i = 0; i < 7; ++i) {
        is >> est[i];
      }
      _estimate = chordal::v2tQUAT(est);
      return true;
    }

    bool VertexSE3Chord::write(std::ostream& os) const {
      Vector7 est = chordal::t2vQUAT(_estimate);
      for (uint8_t i = 0; i < 7; ++i) {
        os << est[i] << " ";
      }
      return os.good();
    }


    //ia actions

    VertexSE3ChordDrawAction::VertexSE3ChordDrawAction() : 
        DrawAction(typeid(VertexSE3Chord).name()) {
      //ia parent ctor
    }

    HyperGraphElementAction* VertexSE3ChordDrawAction::operator()(HyperGraph::HyperGraphElement* element,
                                                                  HyperGraphElementAction::Parameters* params) {
      if (typeid(*element).name()!=_typeName)
        return 0;

      initializeDrawActionsCache();
      refreshPropertyPtrs(params);

      if (! _previousParams)
        return this;
      
      if (_show && !_show->value())
        return this;

      VertexSE3Chord* that = static_cast<VertexSE3Chord*>(element);

      glColor3f(POSE_VERTEX_COLOR);
      glPushMatrix();
      glMultMatrixd(that->estimate().matrix().data());
      opengl::drawArrow2D(_triangleX->value(), _triangleY->value(), _triangleX->value()*.3f);
      drawCache(that->cacheContainer(), params);
      drawUserData(that->userData(), params);
      glPopMatrix();
      return this;
    }


    bool VertexSE3ChordDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params) {
      if (!DrawAction::refreshPropertyPtrs(params))
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
  } //ia end namespace chordal
} //ia end namespace g2o