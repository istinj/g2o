#include <iostream>
#include <fstream>
#include <vector>
#include "g2o/types/slam3d/isometry3d_mappings.h"

#include "g2o/types/matchables3d/matchable.h"


namespace g2o {
  namespace matchables {


    typedef std::vector<std::pair<int, int> > IntIntPairVector;
    typedef std::vector<int> IntVector;
    typedef std::vector<Isometry3> Isometry3Vector;


    class GraphGenerator {
    public:
      
      GraphGenerator();
      virtual ~GraphGenerator();

      void init();

      inline int numLandmarks() const {return _num_landmarks;}
      inline int numPoses() const {return _num_poses;}
      inline void setNumPoses(const int num_poses_) {_num_poses = num_poses_;}
      inline void setNumPoints(const int num_points_) {_num_points = num_points_;}
      inline void setNumLines(const int num_lines_) {_num_lines = num_lines_;}
      inline void setNumPlanes(const int num_planes_) {_num_planes = num_planes_;}
      inline void setWorldSize(const float& world_size_) {_world_size = world_size_;}
      inline void setConstraints(const IntVector& constraints_){_constraints = constraints_;}

      
      void generateLandmarks(MatchableVector& landmarks);
      void generatePoses(Isometry3Vector& poses);
      void generateMeasurements(MatchableMatrix7PairVector& measurements,
                                IntIntPairVector& landmark_associations,
                                const MatchableVector& landmarks,
                                const Isometry3Vector& poses);
      
      void applyPerturbationToData(MatchableVector& landmarks,
                                   Isometry3Vector& poses);

    protected:
      Matchable _generatePointLandmark();
      Matchable _generateLineLandmark();
      Matchable _generatePlaneLandmark();
      MatchableMatrix7Pair _generatePointMeasurementFromLine(const Matchable& landmark,
                                                             const Isometry3& inv_pose);
      MatchableMatrix7Pair _generatePointMeasurementFromPlane(const Matchable& landmark,
                                                              const Isometry3& inv_pose);
      MatchableMatrix7Pair _generateLineMeasurementFromPlane(const Matchable& landmark,
                                                             const Isometry3& inv_pose);
      MatchableMatrix7Pair _generatePointMeasurementFromPoint(const Matchable& landmark,
                                                              const Isometry3& inv_pose);
      MatchableMatrix7Pair _generateLineMeasurementFromLine(const Matchable& landmark,
                                                            const Isometry3& inv_pose);
      MatchableMatrix7Pair _generatePlaneMeasurementFromPlane(const Matchable& landmark,
                                                              const Isometry3& inv_pose);
      
      int _num_points;
      int _num_lines;
      int _num_planes;
      int _num_landmarks;
      int _num_poses;

      int _matchable_dim;
      
      float _world_size;
      IntVector _constraints;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
  }
}
