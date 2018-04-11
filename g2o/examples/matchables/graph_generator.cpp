#include "graph_generator.h"

#include <g2o/types/slam3d/isometry3d_mappings.h>

namespace g2o{
  namespace matchables{

    GraphGenerator::GraphGenerator(int num_points_,
                                   int num_lines_,
                                   int num_planes_,
                                   int num_poses_,
                                   float world_size_):
      _num_points(num_points_),
      _num_lines(num_lines_),
      _num_planes(num_planes_),
      _num_poses(num_poses_),
      _world_size(world_size_){

      _num_landmarks = _num_points + _num_lines + _num_planes;

      _matchable_dim = 13;
    }

    Matchable GraphGenerator::generatePointLandmark(){
      Vector3 p = (Vector3::Random()-0.5*Vector3::Ones())*_world_size;
      Matrix3 R = Matrix3::Identity();
      return Matchable(Matchable::Type::Point,p,R);
    }

    Matchable GraphGenerator::generateLineLandmark(){
      Vector3 p = (Vector3::Random()-0.5*Vector3::Ones())*_world_size;
      Vector3 d = (Vector3::Random()-0.5*Vector3::Ones())*_world_size;
      Matrix3 R = d2R(d);
      return Matchable(Matchable::Type::Line,p,R);
    }

    Matchable GraphGenerator::generatePlaneLandmark(){
      Vector3 p = (Vector3::Random()-0.5*Vector3::Ones())*_world_size;
      Vector3 d = (Vector3::Random()-0.5*Vector3::Ones())*_world_size;
      Matrix3 R = d2R(d);
      return Matchable(Matchable::Type::Plane,p,R);
    }

    std::vector<Matchable> GraphGenerator::generateLandmarks(){
      std::vector<Matchable> landmarks(_num_landmarks);
      for(int i=0; i<_num_points; ++i){
        landmarks[i]=generatePointLandmark();
      }
      for(int i=0; i<_num_lines; ++i){
        landmarks[_num_points+i]=generatePointLandmark();
      }
      for(int i=0; i<_num_planes; ++i){
        landmarks[_num_points+_num_lines+i]=generatePointLandmark();
      }
      return landmarks;
    }

    std::vector<Isometry3> GraphGenerator::generatePoses(){
      Matrix6 rand_scale = Matrix6::Identity();
      rand_scale.block<3,3>(0,0) *= (0.5*_world_size);
      rand_scale.block<3,3>(3,3) *= M_PI;

      std::vector<Isometry3> poses(_num_poses);
      poses[0] = Isometry3::Identity();
      for(int i=1; i<_num_poses; ++i){
        Vector6 v = Vector6::Random()-0.5*Vector6::Ones();
        poses[i] = g2o::internal::fromVectorET(v);
      }

      return poses;
    }

    Matchable GraphGenerator::generatePointMeasurementFromLine(const Matchable & landmark,
                                                               const Isometry3 & inv_pose){
      Matchable prediction = landmark.transform(inv_pose);
      return Matchable(Matchable::Type::Point,prediction.point(),Matrix3::Identity());
    }

    Matchable GraphGenerator::generatePointMeasurementFromPlane(const Matchable &landmark,
                                                                const Isometry3 &inv_pose){
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

      return Matchable(Matchable::Type::Point,Vector3(x(0),x(1),0.0f),Matrix3::Identity());
    }

    Matchable GraphGenerator::generateLineMeasurementFromPlane(const Matchable &landmark,
                                                               const Isometry3 &inv_pose){
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

      return Matchable(Matchable::Type::Line,Vector3(x(0),x(1),0.0f),d2R(nz));
    }

    std::vector<Matchable> GraphGenerator::generateMeasurements(IntIntPairVector &landmark_associations,
                                                                const std::vector<Matchable> &landmarks,
                                                                const std::vector<Isometry3> &poses){

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

      std::vector<Matchable> measurements(num_measurements);

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
            measurements[measurement_idx] = landmark.transform(inv_pose);
            ++measurement_idx;
          }
        }

        //line-point
        if(_constraints[1]){
          for(int landmark_idx=0; landmark_idx<_num_lines; ++landmark_idx){
            Matchable landmark = landmarks[_num_points+landmark_idx];
            landmark_associations[measurement_idx] = std::make_pair(pose_idx,_num_points+landmark_idx);
            measurements[measurement_idx] = generatePointMeasurementFromLine(landmark,inv_pose);
            ++measurement_idx;
          }
        }

        //line-line
        if(_constraints[2]){
          for(int landmark_idx=0; landmark_idx<_num_lines; ++landmark_idx){
            Matchable landmark = landmarks[_num_points+landmark_idx];
            landmark_associations[measurement_idx] = std::make_pair(pose_idx,_num_points+landmark_idx);
            measurements[measurement_idx] = landmark.transform(inv_pose);
            ++measurement_idx;
          }
        }

        //plane-point
        if(_constraints[3]){
          for(int landmark_idx=0; landmark_idx<_num_planes; ++landmark_idx){
            Matchable landmark = landmarks[_num_points+_num_lines+landmark_idx];
            landmark_associations[measurement_idx] = std::make_pair(pose_idx,_num_points+_num_lines+landmark_idx);
            measurements[measurement_idx] = generatePointMeasurementFromPlane(landmark,inv_pose);
            ++measurement_idx;
          }
        }

        //plane-line
        if(_constraints[4]){
          for(int landmark_idx=0; landmark_idx<_num_planes; ++landmark_idx){
            Matchable landmark = landmarks[_num_points+_num_lines+landmark_idx];
            landmark_associations[measurement_idx] = std::make_pair(pose_idx,_num_points+_num_lines+landmark_idx);
            measurements[measurement_idx] = generateLineMeasurementFromPlane(landmark,inv_pose);
            ++measurement_idx;
          }
        }

        //plane-plane
        if(_constraints[5]){
          for(int landmark_idx=0; landmark_idx<_num_planes; ++landmark_idx){
            Matchable landmark = landmarks[_num_points+_num_lines+landmark_idx];
            landmark_associations[measurement_idx] = std::make_pair(pose_idx,_num_points+_num_lines+landmark_idx);
            measurements[measurement_idx] = landmark.transform(inv_pose);
            ++measurement_idx;
          }
        }
      }
      return measurements;
    }

    Matrix3 GraphGenerator::d2R(const Vector3 &d){
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
  }
}
