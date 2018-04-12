#include "matchable_generator.h"

namespace g2o {
  namespace matchables {
   
    Matrix3 d2R(const Vector3& d) {
      Matrix3 R;
      float d_norm = std::sqrt(d(0)*d(0) + d(1)*d(1));
      if(d_norm > 1e-4f)
        R << d(0),d(1)/d_norm,d(0)*d(2)/d_norm,
          d(1),-d(0)/d_norm,d(1)*d(2)/d_norm,
          d(2),0,-d_norm;
      else
        R.setIdentity();

      return R;
    } 

    GraphGenerator::GraphGenerator() {}

    void GraphGenerator::init() {
      _num_landmarks = _num_points + _num_lines + _num_planes;
      _matchable_dim = 13;
    }
            
    GraphGenerator::~GraphGenerator() {}
    
    void GraphGenerator::generateLandmarks(MatchableVector& landmarks) {
      landmarks.clear();
      landmarks.resize(_num_landmarks);
      for(int i=0; i<_num_points; ++i){
        landmarks[i]=_generatePointLandmark();
      }
      for(int i=0; i<_num_lines; ++i){
        landmarks[_num_points+i]=_generateLineLandmark();
      }
      for(int i=0; i<_num_planes; ++i){
        landmarks[_num_points+_num_lines+i]=_generatePlaneLandmark();
      }
    }


    void GraphGenerator::generatePoses(Isometry3Vector& poses){
      Matrix6 rand_scale = Matrix6::Identity();
      rand_scale.block<3,3>(0,0) *= (0.5*_world_size);
      rand_scale.block<3,3>(3,3) *= M_PI;

      poses.clear();
      poses.resize(_num_poses);
      poses[0] = Isometry3::Identity();
      for(int i=1; i<_num_poses; ++i){
        Vector6 v = Vector6::Random()-0.5*Vector6::Ones();
        poses[i] = g2o::internal::fromVectorET(v);
      }
    }

    
    void GraphGenerator::applyPerturbationToData(MatchableVector& landmarks,
                                                 Isometry3Vector& poses){
      float pert_deviation = 1.0f;
      Matrix6 pert_scale = Matrix6::Identity()*pert_deviation;

      for(int i=1; i<_num_poses; ++i) {
        Vector6 xr = Vector6::Random()-0.5*Vector6::Ones();
        Isometry3 dXr = g2o::internal::fromVectorET(pert_scale*xr);
        poses[i] = dXr*poses[i];
      }

      for(int i=0; i<_num_landmarks; ++i){
        Matchable::Vector5 dXl = (Matchable::Vector5::Random()-0.5*Matchable::Vector5::Ones())*pert_deviation;
        landmarks[i] = landmarks[i].perturb(dXl);
      }
    }


    
    void GraphGenerator::generateMeasurements(MatchableMatrix7PairVector& measurements,
                                              IntIntPairVector& landmark_associations,
                                              const MatchableVector& landmarks,
                                              const Isometry3Vector& poses) {

      int num_point_point_measurements=0;
      int num_line_point_measurements=0;
      int num_line_line_measurements=0;
      int num_plane_point_measurements=0;
      int num_plane_line_measurements=0;
      int num_plane_plane_measurements=0;

      if(_constraints[0]) //point-point
        num_point_point_measurements = _num_points;
      if(_constraints[1]) //line-point
        num_line_point_measurements = _num_lines;
      if(_constraints[2]) //line-line
        num_line_line_measurements = _num_lines;
      if(_constraints[3]) //plane-point
        num_plane_point_measurements = _num_planes;
      if(_constraints[4]) //plane-line
        num_plane_line_measurements = _num_planes;
      if(_constraints[5]) //plane-plane
        num_plane_plane_measurements = _num_planes;

      int num_measurements = _num_poses*(num_point_point_measurements+
                                         num_line_point_measurements+
                                         num_line_line_measurements+
                                         num_plane_point_measurements+
                                         num_plane_line_measurements+
                                         num_plane_plane_measurements);

      measurements.clear();
      measurements.resize(num_measurements);

      landmark_associations.clear();
      landmark_associations.resize(num_measurements);

      int measurement_idx = 0;

      for(int pose_idx=0; pose_idx<_num_poses; ++pose_idx){
        const Isometry3 inv_pose = poses[pose_idx].inverse();

        //point-point
        if(_constraints[0]){
          for(int landmark_idx=0; landmark_idx<_num_points; ++landmark_idx){
            Matchable landmark = landmarks[landmark_idx];
            landmark_associations[measurement_idx] = std::make_pair(pose_idx,landmark_idx);
            measurements[measurement_idx] = _generatePointMeasurementFromPoint(landmark, inv_pose);
            ++measurement_idx;
          }
        }

        //line-point
        if(_constraints[1]){
          for(int landmark_idx=0; landmark_idx<_num_lines; ++landmark_idx){
            Matchable landmark = landmarks[_num_points+landmark_idx];
            landmark_associations[measurement_idx] = std::make_pair(pose_idx,_num_points+landmark_idx);
            measurements[measurement_idx] = _generatePointMeasurementFromLine(landmark,inv_pose);
            ++measurement_idx;
          }
        }

        //line-line
        if(_constraints[2]){
          for(int landmark_idx=0; landmark_idx<_num_lines; ++landmark_idx){
            Matchable landmark = landmarks[_num_points+landmark_idx];
            landmark_associations[measurement_idx] = std::make_pair(pose_idx,_num_points+landmark_idx);
            measurements[measurement_idx] = _generateLineMeasurementFromLine(landmark,inv_pose);
            ++measurement_idx;
          }
        }

        //plane-point
        if(_constraints[3]){
          for(int landmark_idx=0; landmark_idx<_num_planes; ++landmark_idx){
            Matchable landmark = landmarks[_num_points+_num_lines+landmark_idx];
            landmark_associations[measurement_idx] = std::make_pair(pose_idx,_num_points+_num_lines+landmark_idx);
            measurements[measurement_idx] = _generatePointMeasurementFromPlane(landmark,inv_pose);
            ++measurement_idx;
          }
        }

        //plane-line
        if(_constraints[4]){
          for(int landmark_idx=0; landmark_idx<_num_planes; ++landmark_idx){
            Matchable landmark = landmarks[_num_points+_num_lines+landmark_idx];
            landmark_associations[measurement_idx] = std::make_pair(pose_idx,_num_points+_num_lines+landmark_idx);
            measurements[measurement_idx] = _generateLineMeasurementFromPlane(landmark,inv_pose);
            ++measurement_idx;
          }
        }

        //plane-plane
        if(_constraints[5]){
          for(int landmark_idx=0; landmark_idx<_num_planes; ++landmark_idx){
            Matchable landmark = landmarks[_num_points+_num_lines+landmark_idx];
            landmark_associations[measurement_idx] = std::make_pair(pose_idx,_num_points+_num_lines+landmark_idx);
            measurements[measurement_idx] = _generatePlaneMeasurementFromPlane(landmark,inv_pose);
            ++measurement_idx;
          }
        }
      }
    }

    //ia protected functions
    Matchable GraphGenerator::_generatePointLandmark() {
      Vector3 p = (Vector3::Random()-0.5*Vector3::Ones())*_world_size;
      Matrix3 R = Matrix3::Identity();
      return Matchable(Matchable::Type::Point,p,R);
    }

    Matchable GraphGenerator::_generateLineLandmark() {
      Vector3 p = (Vector3::Random()-0.5*Vector3::Ones())*_world_size;
      Vector3 d = (Vector3::Random()-0.5*Vector3::Ones())*_world_size;
      Matrix3 R = d2R(d);
      return Matchable(Matchable::Type::Line,p,R);
    }

    Matchable GraphGenerator::_generatePlaneLandmark() {
      Vector3 p = (Vector3::Random()-0.5*Vector3::Ones())*_world_size;
      Vector3 d = (Vector3::Random()-0.5*Vector3::Ones())*_world_size;
      Matrix3 R = d2R(d);
      return Matchable(Matchable::Type::Plane,p,R);
    }
    
    MatchableMatrix7Pair GraphGenerator::_generatePointMeasurementFromPoint(const Matchable& landmark,
                                                                            const Isometry3& inv_pose) {
      MatchableMatrix7Pair z;
      z.first = landmark.transform(inv_pose);
      z.second = Matrix7::Zero();
      z.second.block<3,3>(0,0) = Matrix3::Identity();
      return z;
    }

    MatchableMatrix7Pair GraphGenerator::_generatePointMeasurementFromLine(const Matchable& landmark,
                                                                           const Isometry3& inv_pose) {
      Matchable prediction = landmark.transform(inv_pose);
      MatchableMatrix7Pair z;

      z.first = Matchable(Matchable::Type::Point,prediction.point(),Matrix3::Identity());
      z.second = Matrix7::Zero();
      z.second.block<3,3>(0,0) = landmark.omega();
      
      return z;
    }

        
    MatchableMatrix7Pair GraphGenerator::_generateLineMeasurementFromLine(const Matchable& landmark,
                                                                          const Isometry3& inv_pose) {
      MatchableMatrix7Pair z;
      z.first = landmark.transform(inv_pose);
      z.second = Matrix7::Zero();
      z.second.block<3,3>(0,0) = landmark.omega();
      z.second.block<3,3>(3,3) = Matrix3::Identity();
      return z;
    }

    MatchableMatrix7Pair GraphGenerator::_generatePointMeasurementFromPlane(const Matchable& landmark,
                                                                 const Isometry3& inv_pose) {
      const Vector3 &p = inv_pose.translation();
      const Vector3 &n = inv_pose.linear().col(2);
      const float d  = -n.transpose()*p;

      Matchable prediction = landmark.transform(inv_pose);
      const Vector3 &pl = prediction.point();
      const Vector3 &nl = prediction.R().col(0);
      const float dl  = -nl.transpose()*pl;

      Matrix2 A;
      Vector2 b;
      A << n(0),n(1),nl(0),nl(1);
      b << -d,-dl;
      Vector2 x = A.colPivHouseholderQr().solve(b);

      MatchableMatrix7Pair z;
      z.first = Matchable(Matchable::Type::Point,Vector3(x(0),x(1),0.0f),Matrix3::Identity());
      z.second = Matrix7::Zero();
      z.second.block<3,3>(0,0) = landmark.omega();

      return z;
    }

    MatchableMatrix7Pair GraphGenerator::_generateLineMeasurementFromPlane(const Matchable& landmark,
                                                                const Isometry3& inv_pose) {
      const Vector3 &p = inv_pose.translation();
      const Vector3 &n = inv_pose.linear().col(2);
      const float d  = -n.transpose()*p;

      Matchable prediction = landmark.transform(inv_pose);
      const Vector3 &pl = prediction.point();
      const Vector3 &nl = prediction.R().col(0);
      const float dl  = -nl.transpose()*pl;

      Matrix2 A;
      Vector2 b;
      A << n(0),n(1),nl(0),nl(1);
      b << -d,-dl;
      Vector2 x = A.colPivHouseholderQr().solve(b);

      Vector3 nz = n.cross(nl);
      nz.normalize();

      MatchableMatrix7Pair z;
      z.first = Matchable(Matchable::Type::Line,Vector3(x(0),x(1),0.0f),d2R(nz));
      z.second = Matrix7::Zero();
      z.second.block<3,3>(0,0) = landmark.omega();
      z.second(6,6) = 1;

      return z;
    }

    MatchableMatrix7Pair GraphGenerator::_generatePlaneMeasurementFromPlane(const Matchable& landmark,
                                                                            const Isometry3& inv_pose) {
      MatchableMatrix7Pair z;
      z.first = landmark.transform(inv_pose);
      z.second = Matrix7::Zero();
      z.second.block<3,3>(0,0) = landmark.omega();
      z.second.block<3,3>(3,3) = Matrix3::Identity();
      return z;
    }
    
  } //ia end matchable namespace
} //ia end g2o namespace
