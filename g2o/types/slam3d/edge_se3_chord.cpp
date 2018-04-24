#include <iomanip>
#include <iostream>
#include "edge_se3_chord.h"
#include "isometry3d_mappings.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/stuff/opengl_primitives.h"
#endif

namespace g2o {
  EdgeSE3Chord::EdgeSE3Chord() :
    BaseBinaryEdge<12, Isometry3, VertexSE3Chord, VertexSE3Chord>() {
    _information.setIdentity();
  }

   
  bool EdgeSE3Chord::read(std::istream& is) {
    Vector7 meas = Vector7::Zero();
    for (uint8_t i = 0; i < 7; ++i) {
      is >> meas[i];
    }

    _measurement = internal::fromVectorQT(meas);

    // _information.block<9,9>(0,0) = _information.block<9,9>(0,0) * 1e10;

    for (int i = 0; i < _information.rows(); ++i) {
      for (int j = i; j < _information.cols(); ++j) {
        is >> _information(i,j);
        if (i != j)
          _information(j,i) = _information(i,j);
      }
    }

    return true;
  }

    
  bool EdgeSE3Chord::write(std::ostream& os) const {
    Vector7 meas = internal::toVectorQT(_measurement);
    for (uint8_t i = 0; i < 7; ++i) {
      os << meas[i] << " ";
    }

    for (int r = 0; r < _information.rows(); ++r) {
      for (int c = r; c < _information.cols(); ++c) {
        os << _information(r,c) << " ";
      }
    }
    return os.good();
  }

    
  void EdgeSE3Chord::computeError() {
    VertexSE3Chord* v_from = static_cast<VertexSE3Chord*>(_vertices[0]);
    VertexSE3Chord* v_to   = static_cast<VertexSE3Chord*>(_vertices[1]);

    //! Prediction
    Isometry3 h_x = v_from->estimate().inverse() * v_to->estimate();

    //! Error
    Isometry3 delta = Isometry3::Identity();
    delta.matrix() = h_x.matrix() - _measurement.matrix();
    _error = internal::toFlatten(delta);
  }

    
  void EdgeSE3Chord::linearizeOplus() {
    VertexSE3Chord* v_from = static_cast<VertexSE3Chord*>(_vertices[0]);
    VertexSE3Chord* v_to   = static_cast<VertexSE3Chord*>(_vertices[1]);

    _jacobianOplusXi.setZero();
    _jacobianOplusXj.setZero();

    const Isometry3& pose_i = v_from->estimate();
    const Isometry3& pose_j = v_to->estimate();

    Matrix3 Rx0, Ry0, Rz0;
    Rx0 << 0,0,0,  0,0,-1,  0,1,0;
    Ry0 << 0,0,1,  0,0,0,   -1,0,0;
    Rz0 << 0,-1,0, 1,0,0,   0,0,0;

    Matrix3 Ri = pose_i.linear();

    Matrix3 Rj = pose_j.linear();
    Vector3 tj = pose_j.translation();

    Matrix3 dR_x = Ri.transpose() * Rx0 * Rj;
    Matrix3 dR_y = Ri.transpose() * Ry0 * Rj;
    Matrix3 dR_z = Ri.transpose() * Rz0 * Rj;

    Vector9 dr_x_flattened, dr_y_flattened, dr_z_flattened;
    dr_x_flattened << dR_x.col(0), dR_x.col(1), dR_x.col(2);
    dr_y_flattened << dR_y.col(0), dR_y.col(1), dR_y.col(2);
    dr_z_flattened << dR_z.col(0), dR_z.col(1), dR_z.col(2);

    //! Fill Jj
    _jacobianOplusXj.block<9,1>(0,3) = dr_x_flattened;
    _jacobianOplusXj.block<9,1>(0,4) = dr_y_flattened;
    _jacobianOplusXj.block<9,1>(0,5) = dr_z_flattened;
    _jacobianOplusXj.block<3,3>(9,0) = Ri.transpose();
    _jacobianOplusXj.block<3,3>(9,3) = -Ri.transpose() * skew(tj);

    _jacobianOplusXi = -_jacobianOplusXj;
  }

  bool EdgeSE3Chord::setMeasurementFromState(){
    VertexSE3Chord* from = static_cast<VertexSE3Chord*>(_vertices[0]);
    VertexSE3Chord* to   = static_cast<VertexSE3Chord*>(_vertices[1]);
    Isometry3 delta = from->estimate().inverse() * to->estimate();
    setMeasurement(delta);
    return true;
  }

  void EdgeSE3Chord::initialEstimate(const OptimizableGraph::VertexSet& from_, OptimizableGraph::Vertex* /*to_*/) {
    VertexSE3Chord* from = static_cast<VertexSE3Chord*>(_vertices[0]);
    VertexSE3Chord* to = static_cast<VertexSE3Chord*>(_vertices[1]);
    if (from_.count(from) > 0) {
      to->setEstimate(from->estimate() * _measurement);
    } else
      from->setEstimate(to->estimate() * _measurement.inverse());
  }



  EdgeSE3ChordDrawAction::EdgeSE3ChordDrawAction() : DrawAction(typeid(EdgeSE3Chord).name()) {
    //ia parent ctor
  }

  HyperGraphElementAction* EdgeSE3ChordDrawAction::operator()(HyperGraph::HyperGraphElement* element,
                                                              HyperGraphElementAction::Parameters* params) {
    if (typeid(*element).name()!=_typeName)
      return 0;
    refreshPropertyPtrs(params);
    if (! _previousParams)
      return this;
      
    if (_show && !_show->value())
      return this;
      
    EdgeSE3Chord* e =  static_cast<EdgeSE3Chord*>(element);
    VertexSE3Chord* fromEdge = static_cast<VertexSE3Chord*>(e->vertices()[0]);
    VertexSE3Chord* toEdge   = static_cast<VertexSE3Chord*>(e->vertices()[1]);
    if (! fromEdge || ! toEdge)
      return this;
    glColor3f(POSE_EDGE_COLOR);
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glVertex3f((float)fromEdge->estimate().translation().x(),(float)fromEdge->estimate().translation().y(),(float)fromEdge->estimate().translation().z());
    glVertex3f((float)toEdge->estimate().translation().x(),(float)toEdge->estimate().translation().y(),(float)toEdge->estimate().translation().z());
    glEnd();
    glPopAttrib();
    return this;
  }
} //ia end namespace g2o
