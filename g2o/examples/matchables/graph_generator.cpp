#include <iostream>
#include <fstream>
#include <vector>
#include "g2o/types/matchables3d/matchable.h"

#include <g2o/types/slam3d/isometry3d_mappings.h>

namespace g2o{
  namespace matchables{

    class GraphGenerator{
    public:

      typedef Eigen::Matrix<number_t,6,6,Eigen::ColMajor> Matrix6;
      typedef Eigen::Matrix<number_t,7,7,Eigen::ColMajor> Matrix7;
      typedef std::vector<std::pair<int,int> > IntIntPairVector;

      GraphGenerator(int num_points_ = 0,
                     int num_lines_ = 0,
                     int num_planes_ = 0,
                     int num_poses_ = 0,
                     float world_size_ = 0):
        _num_points(num_points_),
        _num_lines(num_lines_),
        _num_planes(num_planes_),
        _num_poses(num_poses_),
        _world_size(world_size_){

        _num_landmarks = _num_points + _num_lines + _num_planes;

        _matchable_dim = 13;
      }

      inline int numPoses() const {return _num_poses;}

      inline void setConstraints(const std::vector<int> &constraints_){_constraints = constraints_;}

      void generateLandmarks(std::vector<Matchable> &landmarks){
        landmarks.clear();
        landmarks.resize(_num_landmarks);
        for(int i=0; i<_num_points; ++i){
          landmarks[i]=generatePointLandmark();
        }
        for(int i=0; i<_num_lines; ++i){
          landmarks[_num_points+i]=generateLineLandmark();
        }
        for(int i=0; i<_num_planes; ++i){
          landmarks[_num_points+_num_lines+i]=generatePlaneLandmark();
        }
      }

      void generatePoses(std::vector<Isometry3> &poses){
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

      void generateMeasurements(std::vector<Matchable> &measurements,
                                IntIntPairVector &landmark_associations,
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
      }

      void applyPerturbationToData(std::vector<Matchable> &landmarks,
                                   std::vector<Isometry3> &poses){
        float pert_deviation = 1.0f;
        Matrix6 pert_scale = Matrix6::Identity()*pert_deviation;

        for(int i=1; i<_num_poses; ++i){
          Vector6 xr = Vector6::Random()-0.5*Vector6::Ones();
          Isometry3 dXr = g2o::internal::fromVectorET(pert_scale*xr);
          poses[i] = dXr*poses[i];
        }

        for(int i=0; i<_num_landmarks; ++i){
          Matchable::Vector5 dXl = (Matchable::Vector5::Random()-0.5*Matchable::Vector5::Ones())*pert_deviation;
          landmarks[i] = landmarks[i].perturb(dXl);
        }
      }

    protected:
      int _num_points;
      int _num_lines;
      int _num_planes;
      int _num_landmarks;
      int _num_poses;
      float _world_size;
      std::vector<int> _constraints;

      int _matchable_dim;

    private:
      Matchable generatePointLandmark(){
        Vector3 p = (Vector3::Random()-0.5*Vector3::Ones())*_world_size;
        Matrix3 R = Matrix3::Identity();
        return Matchable(Matchable::Type::Point,p,R);
      }

      Matchable generateLineLandmark(){
        Vector3 p = (Vector3::Random()-0.5*Vector3::Ones())*_world_size;
        Vector3 d = (Vector3::Random()-0.5*Vector3::Ones())*_world_size;
        Matrix3 R = d2R(d);
        return Matchable(Matchable::Type::Line,p,R);
      }

      Matchable generatePlaneLandmark(){
        Vector3 p = (Vector3::Random()-0.5*Vector3::Ones())*_world_size;
        Vector3 d = (Vector3::Random()-0.5*Vector3::Ones())*_world_size;
        Matrix3 R = d2R(d);
        return Matchable(Matchable::Type::Plane,p,R);
      }

      Matchable generatePointMeasurementFromLine(const Matchable &landmark, const Isometry3 &inv_pose){
        Matchable prediction = landmark.transform(inv_pose);
        return Matchable(Matchable::Type::Point,prediction.point(),Matrix3::Identity());
      }

      Matchable generatePointMeasurementFromPlane(const Matchable &landmark, const Isometry3 &inv_pose){
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

      Matchable generateLineMeasurementFromPlane(const Matchable &landmark, const Isometry3 &inv_pose){
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

      Matrix3 d2R(const Vector3 &d){
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
    };


  }
}

using namespace g2o;
using namespace matchables;

int main(){

  int num_points=0;
  int num_lines=10;
  int num_planes=0;
  int num_poses=10;
  float world_size=10;

  std::vector<int> constraints(6);

  constraints[0] = 0; //point-point
  constraints[1] = 0; //line-point
  constraints[2] = 1; //line-line
  constraints[3] = 0; //plane-point
  constraints[4] = 0; //plane-line
  constraints[5] = 1; //plane-plane

  GraphGenerator generator(num_points,num_lines,num_planes,num_poses,world_size);
  generator.setConstraints(constraints);

  //generate landmarks
  std::vector<Matchable> landmarks;
  generator.generateLandmarks(landmarks);

  //generate poses
  std::vector<Isometry3> poses;
  generator.generatePoses(poses);

  //generate measurements
  GraphGenerator::IntIntPairVector landmark_associations;
  std::vector<Matchable> measurements;
  generator.generateMeasurements(measurements,landmark_associations,landmarks,poses);

  //generate wrong initial guess
  // generator.applyPerturbationToData(landmarks,poses);

  //write to file
  const char* filename = "graph.g2o";
  std::ofstream graph(filename);

  int id=0;
  
  for(size_t i=0; i<poses.size(); ++i){
    const Isometry3 &pose = poses[i];
    graph << "VERTEX_SE3:CHORD " << id << " ";
    const Vector7 v = internal::toVectorQT(pose);
    for(size_t j=0; j<7; ++j)
      graph << v[j] << " ";
    graph << std::endl;
    ++id;
  }


  for(size_t i=0; i<landmarks.size(); ++i){
    const Matchable &landmark = landmarks[i];
    graph << "VERTEX_MATCHABLE " << id << " ";
    const Matchable::Vector13 v = landmark.toVector();
    for(size_t j=0; j<13; ++j)
      graph << v[j] << " ";
    graph << std::endl;
    ++id;
  }

  GraphGenerator::Matrix7 omega = GraphGenerator::Matrix7::Identity();
  
  for(size_t i=0; i<measurements.size(); ++i){
    Matchable measurement = measurements[i];
    graph << "EDGE_SE3_MATCHABLE " 
          << landmark_associations[i].first << " "
          << generator.numPoses()+landmark_associations[i].second << " ";
    
    Matchable::Vector13 v = measurement.toVector();
    for(size_t j=0; j<13; ++j)
      graph << v[j] << " ";

    for (int r = 0; r < omega.rows(); ++r) {
      for (int c = r; c < omega.cols(); ++c) {
        graph << omega(r,c) << " ";
      }
    }
    
    graph << std::endl;
    ++id;
  }

  graph.close();

  return 0;
}
