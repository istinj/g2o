#pragma once
#include <vector>

#include "matchable.h"

namespace g2o{
  namespace matchables{
    class GraphGenerator{
    public:

      typedef Eigen::Matrix<number_t,6,6,Eigen::ColMajor> Matrix6;
      typedef std::vector<std::pair<int,int> > IntIntPairVector;

      GraphGenerator(int num_points_ = 0,
                     int num_lines_ = 0,
                     int num_planes_ = 0,
                     int num_poses_ = 0,
                     float world_size_ = 0);

      inline void setConstraints(const std::vector<int> &constraints_){_constraints = constraints_;}

      std::vector<Matchable> generateLandmarks();

      std::vector<Isometry3> generatePoses();

      std::vector<Matchable> generateMeasurements(IntIntPairVector &landmark_associations,
                                                  const std::vector<Matchable> &landmarks,
                                                  const std::vector<Isometry3> &poses);



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
      Matchable generatePointLandmark();
      Matchable generateLineLandmark();
      Matchable generatePlaneLandmark();

      Matchable generatePointMeasurementFromLine(const Matchable &landmark, const Isometry3 &inv_pose);
      Matchable generatePointMeasurementFromPlane(const Matchable &landmark, const Isometry3 &inv_pose);
      Matchable generateLineMeasurementFromPlane(const Matchable &landmark, const Isometry3 &inv_pose);

      Matrix3 d2R(const Vector3 &d);

    };
  }
}
