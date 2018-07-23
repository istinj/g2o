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
      is >> R(0,0) >> R(0,1) >> R(0,2)
         >> R(1,0) >> R(1,1) >> R(1,2)
         >> R(2,0) >> R(2,1) >> R(2,2);

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
        throw std::runtime_error("[EdgeSE3Matchable][Read] unknown matchable type");
      }

      setMeasurement(Matchable(type,p,R));

      if (is.bad())
        return false;

      for (int i = 0; i < _information.rows() && is.good(); i++)
        for (int j = i; j < _information.cols() && is.good(); j++){
          is >> _information(i,j);
          if (i!=j)
            _information(j,i)=_information(i,j);
        }

      if (is.bad())
        _information.setIdentity();

      return true;
    }

    bool EdgeSE3Matchable::write(std::ostream &os) const{
      const Vector3& p = _measurement.point();
      const Matrix3& R = _measurement.rotation();

      os << _measurement.type() << " ";
      os << p[0] << " " << p[1] << " " << p[2] << " ";
      os << R(0,0) << " " << R(0,1) << " " << R(0,2) << " "
         << R(1,0) << " " << R(1,1) << " " << R(1,2) << " "
         << R(2,0) << " " << R(2,1) << " " << R(2,2) << " ";

      for (int i = 0; i < _information.rows(); i++)
        for (int j = i; j < _information.cols(); j++)
          os << _information(i,j) << " ";

      return os.good();
    }

    void EdgeSE3Matchable::computeError() {

      VertexSE3Chord* v_from = static_cast<VertexSE3Chord*>(_vertices[0]);
      VertexMatchable* v_to = static_cast<VertexMatchable*>(_vertices[1]);

      const Isometry3& pose = v_from->estimate();
      const Vector3& t = pose.translation();
      const Matrix3& R = pose.linear();

      const Vector3& pl = v_to->estimate().point();
      const Matrix3& Rl = v_to->estimate().rotation();

      const Vector3& pz = _measurement.point();
      const Matrix3& Rz = _measurement.rotation();

      const Vector3  ep = Rl.transpose()*(R*pz + t - pl);
      const Vector3  ed = R*Rz.col(0) - Rl.col(0);
      const number_t eo = (R*Rz).col(0).transpose() * Rl.col(0);

      _error.block<3,1>(0,0) = ep;
      _error.block<3,1>(3,0) = ed;
      _error[6] = eo;
    }


    void EdgeSE3Matchable::numericalJacobians(Eigen::Matrix<number_t, 7, 6>& Ji_,
                                              Eigen::Matrix<number_t, 7, 5>& Jj_) {
      VertexSE3Chord* vi = static_cast<VertexSE3Chord*>(_vertices[0]);
      VertexMatchable* vj = static_cast<VertexMatchable*>(_vertices[1]);

      const number_t epsilon(1e-9);
      const number_t delta = 1.0/(2*epsilon);
      
      Ji_.setZero();
      Jj_.setZero();

      // const int& vi_dimension = vi->minimalEstimateDimension();
      // const int& vj_dimension = vj->minimalEstimateDimension(); //ia why I get -1??
      const int& vi_dimension = 6;
      const int& vj_dimension = 5;

      // std::cerr << "vi_dimension=" << vi_dimension << std::endl;
      // std::cerr << "vj_dimension=" << vj_dimension << std::endl;

      const Vector7 error_backup = _error;


      //ia pose
      Vector7 error_i = Vector7::Zero();
      number_t dxi[vi_dimension] = {};
      for (int i = 0; i < vi_dimension; ++i) {
        vi->push();
        dxi[i] = epsilon;
        vi->oplus(dxi);
        computeError();
        error_i = _error;
        vi->pop();


        vi->push();
        dxi[i] = -epsilon;
        vi->oplus(dxi);
        computeError();
        error_i -= _error;
        vi->pop();

        dxi[i] = 0.0;
        Ji_.col(i) = delta * error_i;
      }
      
      //ia matchable
      Vector7 error_j = Vector7::Zero();      
      number_t dxj[vj_dimension] = {};
      for (int i = 0; i < vj_dimension; ++i) {
        vj->push();
        dxj[i] = epsilon;
        vj->oplus(dxj);
        computeError();
        error_j = _error;
        vj->pop();


        vj->push();
        dxj[i] = -epsilon;
        vj->oplus(dxj);
        computeError();
        error_j -= _error;
        vj->pop();

        dxj[i] = 0.0;
        Jj_.col(i) = delta * error_j;
      }

      _error = error_backup;
      
    }


    void EdgeSE3Matchable::analyticalJacobians(Eigen::Matrix<number_t, 7, 6>& Ji_,
                                               Eigen::Matrix<number_t, 7, 5>& Jj_) {
      VertexSE3Chord* v_from = static_cast<VertexSE3Chord*>(_vertices[0]);
      VertexMatchable* v_to = static_cast<VertexMatchable*>(_vertices[1]);

      Ji_.setZero();
      Jj_.setZero();

      const Isometry3& pose = v_from->estimate();
      const Vector3& t = pose.translation();
      const Matrix3& R = pose.linear();

      const Vector3& pl = v_to->estimate().point();
      const Matrix3& Rl = v_to->estimate().rotation();

      const Vector3& pz = _measurement.point();
      const Matrix3& Rz = _measurement.rotation();
      
      //ia compute Ji - components
      Matrix3 dep_dt = Rl.transpose();
      Matrix3 dep_dR = -Rl.transpose() * skew(R*pz + t);

      // Matrix3 ded_dt = Matrix3::Zero();
      Matrix3 ded_dR = -skew((R*Rz).col(0));

      // Vector3 deo_dt = Vector3::Zero();
      Matrix3 deo_dR_matrix = Rz.transpose() * R.transpose() * skew(Rl.col(0));
      Vector3 deo_dR = deo_dR_matrix.row(0);

      Ji_.block<3,3>(0,0) = dep_dt;
      Ji_.block<3,3>(0,3) = dep_dR;
      Ji_.block<3,3>(3,3) = ded_dR;
      Ji_.block<1,3>(6,3) = deo_dR.transpose();

      //ia compute Jj - components
      Matrix3 dep_dpl = -Rl.transpose();
      Eigen::Matrix<number_t, 3, 2> dep_dRl = skew(Rl.transpose() * (R*pz + t - pl)).block<3,2>(0,1);

      // Matrix3 ded_dpl = Matrix3::Zero();
      Eigen::Matrix<number_t, 3, 2> ded_dRl = Rl*skew(Vector3::UnitX()).block<3,2>(0,1);

      // Vector3 deo_dpl = Vector3::Zero();
      Eigen::Matrix<number_t, 3, 2> deo_dRl_matrix =
        Rz.transpose() * R.transpose() * Rl * skew(Vector3::UnitX()).block<3,2>(0,1);
      Vector2 deo_dRl = -deo_dRl_matrix.row(0);

      Jj_.block<3,3>(0,0) = dep_dpl;
      Jj_.block<3,2>(0,3) = dep_dRl;
      Jj_.block<3,2>(3,3) = ded_dRl;
      Jj_.block<1,2>(6,3) = deo_dRl.transpose();


      // std::cerr << "Ji_ size: "
      //           << Ji_.rows() << "x"
      //           << Ji_.cols() << std::endl
      //           << Ji_ << std::endl;
      
      // std::cerr << "Jj_ size: "
      //           << Jj_.rows() << "x"
      //           << Jj_.cols() << std::endl
      //           << Jj_ << std::endl;
      // std::cin.get();
      
    }

    /*
    void EdgeSE3Matchable::linearizeOplus() {
      VertexSE3Chord* v_from = static_cast<VertexSE3Chord*>(_vertices[0]);
      VertexMatchable* v_to = static_cast<VertexMatchable*>(_vertices[1]);

      _jacobianOplusXi.setZero();
      _jacobianOplusXj.setZero();

      const Isometry3& pose = v_from->estimate();
      const Vector3& t = pose.translation();
      const Matrix3& R = pose.linear();

      const Vector3& pl = v_to->estimate().point();
      const Matrix3& Rl = v_to->estimate().rotation();

      const Vector3& pz = _measurement.point();
      const Matrix3& Rz = _measurement.rotation();
      
      //ia compute Ji - components
      Matrix3 dep_dt = Rl.transpose();
      Matrix3 dep_dR = -Rl.transpose() * skew(R*pz + t);

      // Matrix3 ded_dt = Matrix3::Zero();
      Matrix3 ded_dR = -skew((R*Rz).col(0));

      // Vector3 deo_dt = Vector3::Zero();
      Matrix3 deo_dR_matrix = Rz.transpose() * R.transpose() * skew(Rl.col(0));
      Vector3 deo_dR = deo_dR_matrix.row(0);

      _jacobianOplusXi.block<3,3>(0,0) = dep_dt;
      _jacobianOplusXi.block<3,3>(0,3) = dep_dR;
      _jacobianOplusXi.block<3,3>(3,3) = ded_dR;
      _jacobianOplusXi.block<1,3>(6,3) = deo_dR.transpose();

      //ia compute Jj - components
      Matrix3 dep_dpl = -Rl.transpose();
      Eigen::Matrix<number_t, 3, 2> dep_dRl = skew(Rl.transpose() * (R*pz + t - pl)).block<3,2>(0,1);

      // Matrix3 ded_dpl = Matrix3::Zero();
      Eigen::Matrix<number_t, 3, 2> ded_dRl = Rl*skew(Vector3::UnitX()).block<3,2>(0,1);

      // Vector3 deo_dpl = Vector3::Zero();
      Eigen::Matrix<number_t, 3, 2> deo_dRl_matrix =
        Rz.transpose() * R.transpose() * Rl * skew(Vector3::UnitX()).block<3,2>(0,1);
      Vector2 deo_dRl = -deo_dRl_matrix.row(0);

      _jacobianOplusXj.block<3,3>(0,0) = dep_dpl;
      _jacobianOplusXj.block<3,2>(0,3) = dep_dRl;
      _jacobianOplusXj.block<3,2>(3,3) = ded_dRl;
      _jacobianOplusXj.block<1,2>(6,3) = deo_dRl.transpose();

      std::cerr << "[analytic linearization] of edge #" 
                << v_from->id() << "->" 
                << v_to->id() << std::endl;
      std::cerr << "_jacobianOplusXi size: "
                << _jacobianOplusXi.rows() << "x"
                << _jacobianOplusXi.cols() << std::endl
                << _jacobianOplusXi << std::endl;
      
      std::cerr << "_jacobianOplusXj size: "
                << _jacobianOplusXj.rows() << "x"
                << _jacobianOplusXj.cols() << std::endl
                << _jacobianOplusXj << std::endl;
      std::cerr << std::endl;
      // std::cin.get();
    }/**/


    void EdgeSE3Matchable::initialEstimate(const OptimizableGraph::VertexSet& /*from*/,
                                           OptimizableGraph::Vertex* /*to*/) {
      //ia how do I compute the fucking initial guess??
      //ia moreover, this initial guess will produce a non zero chi2
      //ia also when there is no error encoded in the measurement.
      //ia this happens only in using the non-homogeneous factors
      VertexSE3Chord* v_from = static_cast<VertexSE3Chord*>(_vertices[0]);
      VertexMatchable* v_to  = static_cast<VertexMatchable*>(_vertices[1]);

      Matchable new_estimate =
        _measurement.applyTransform(v_from->estimate());
      
      v_to->setEstimate(new_estimate);
    }



#ifdef G2O_HAVE_OPENGL
    EdgeSE3MatchableDrawAction::EdgeSE3MatchableDrawAction(): DrawAction(typeid(EdgeSE3Matchable).name()){}

    HyperGraphElementAction* EdgeSE3MatchableDrawAction::operator()(HyperGraph::HyperGraphElement* element,
                                                                    HyperGraphElementAction::Parameters* params_) {
      
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
      glVertex3f((float)fromTransform.translation().x(),
                 (float)fromTransform.translation().y(),
                 (float)fromTransform.translation().z());
      glVertex3f((float)toEdge->estimate().point().x(),
                 (float)toEdge->estimate().point().y(),
                 (float)toEdge->estimate().point().z());
      glEnd();
      glPopAttrib();
      return this;
    }
#endif

  }
}
