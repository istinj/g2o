#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <cmath>

//ia include types
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"
#include "g2o/types/matchables3d/types_matchables.h"

//ia include g2o core stuff
#include "g2o/stuff/sampler.h"
#include "g2o/stuff/command_args.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/factory.h"

#include <Eigen/Core>
#include <Eigen/StdVector>

using namespace std;
using namespace g2o;
using namespace matchables;

void addMatchableNoise(EdgeSE3Matchable* edge_,
                       GaussianSampler<Vector3, Matrix3>& p_sampler_,
                       GaussianSampler<Vector2, Matrix2>& n_sampler_,
                       const Matrix3& point_sigma_,
                       const Matrix2& normal_sigma_);

void addPoseNoise(EdgeSE3Chord* edge_,
                  GaussianSampler<Vector3, Matrix3>& t_sampler_,
                  GaussianSampler<Vector3, Matrix3>& r_sampler_,
                  const Matrix3& translational_sigma_,
                  const Matrix3& rotational_sigma_);

int main(int argc, char** argv) {
  std::cerr << "graph must be in a OPTMIMUM STATE" << std::endl << std::endl;

  //ia for the factory
  VertexSE3Chord v_se3;
  VertexMatchable v_match;
  EdgeSE3Chord e_se3;
  EdgeSE3Matchable e_match;
  std::string e_matchable_tag = Factory::instance()->tag(&e_match);
  std::string e_se3_tag = Factory::instance()->tag(&e_se3);

  //ia input params
  std::string input_filename;
  std::string output_filename;
  
  std::vector<number_t> normal_noise;  //ia noise on the matchable normal
  std::vector<number_t> point_noise;   //ia noise on the matchable point

  std::vector<number_t> translational_noise;  //ia noise on the robot position
  std::vector<number_t> rotational_noise;     //ia noise on the robot rotation

  g2o::CommandArgs arg;
  arg.param("o", output_filename, "", "output final version of the graph");
  arg.param("pointNoise", point_noise,
            std::vector<number_t>(3,0.005),
            "translational noise as <nx,ny,nz>.");
  arg.param("normalNoise", normal_noise,
            std::vector<number_t>(2,0.001),
            "matchable normal noise as <ny,nz>.");
  arg.param("translationNoise", translational_noise,
            std::vector<number_t>(3,0.005),
            "odometry noise for translation as <nx,ny,nz>.");
  arg.param("rotationNoise", rotational_noise,
            std::vector<number_t>(3,0.001),
            "odometry noise for rotation as <nr,np,ny>");
  arg.paramLeftOver("graph-input", input_filename, "", "input file (already at the optimum)", true);
  arg.parseArgs(argc, argv);

  //ia generate noise distrubuions for the matchables
  Matrix3 translation_noise_sigma = Matrix3::Zero();
  Matrix3 rotation_noise_sigma = Matrix3::Zero();

  std::cerr << "Noise for the odometry translation:";
  for (size_t i = 0; i < translational_noise.size(); ++i) {
    std::cerr << " " << translational_noise[i];
    translation_noise_sigma(i, i) = std::pow(translational_noise[i], 2);
  }
  std::cerr << std::endl;

  std::cerr << "Noise for the odometry rotation:";
  for (size_t i = 0; i < rotational_noise.size(); ++i) {
    std::cerr << " " << rotational_noise[i];
    rotation_noise_sigma(i, i) = std::pow(rotational_noise[i], 2);
  }
  std::cerr << std::endl;

  //ia generate noise distrubuions for the matchables
  Matrix3 point_noise_sigma = Matrix3::Zero();
  Matrix2 normal_noise_sigma = Matrix2::Zero();

  std::cerr << "Noise for the matchable point:";
  for (size_t i = 0; i < point_noise.size(); ++i) {
    std::cerr << " " << point_noise[i];
    point_noise_sigma(i, i) = std::pow(point_noise[i], 2);
  }
  std::cerr << std::endl;

  std::cerr << "Noise for the matchable normal:";
  for (size_t i = 0; i < normal_noise.size(); ++i) {
    std::cerr << " " << normal_noise[i];
    normal_noise_sigma(i, i) = std::pow(normal_noise[i], 2);
  }
  std::cerr << std::endl;

  //ia gaussian samplers setup
  GaussianSampler<Vector3, Matrix3> p_sampler; //ia matchable point
  GaussianSampler<Vector2, Matrix2> n_sampler; //ia matchable normal
  GaussianSampler<Vector3, Matrix3> t_sampler; //ia odometry translation
  GaussianSampler<Vector3, Matrix3> r_sampler; //ia odometry rotation

  p_sampler.setDistribution(point_noise_sigma);
  n_sampler.setDistribution(normal_noise_sigma);
  t_sampler.setDistribution(translation_noise_sigma);
  r_sampler.setDistribution(rotation_noise_sigma);

  //ia load the graph
  g2o::SparseOptimizer optimizer;
  std::cerr << "opening file : " << input_filename << std::endl;
  std::ifstream ifs(input_filename.c_str());
  optimizer.load(ifs);

  g2o::HyperGraph::EdgeSet& edges = optimizer.edges();
  const size_t num_vertices = optimizer.vertices().size();
  const size_t num_edges = optimizer.edges().size();
  std::cerr << "read " << num_vertices << " vertices and "
            << num_edges << " edges" << std::endl << std::endl;

  size_t cnt = 0;
  std::cerr << "applying noise to the edges" << std::endl;
  for (g2o::HyperGraph::Edge* e : edges) {
    float percentage = std::ceil((float)cnt++ / (float)num_edges * 100);
    if ((int)percentage % 5 == 0)
      std::cerr << "\rcompleted " << percentage << "%" << std::flush;
    
    if (e_matchable_tag == Factory::instance()->tag(e)) {
      EdgeSE3Matchable* e_matchable = dynamic_cast<EdgeSE3Matchable*>(e);
      addMatchableNoise(e_matchable,
                        p_sampler, n_sampler,
                        point_noise_sigma, normal_noise_sigma);
      continue;
    }
    
    if (e_se3_tag == Factory::instance()->tag(e)) {
      EdgeSE3Chord* e_se3_chordal = dynamic_cast<EdgeSE3Chord*>(e);
      addPoseNoise(e_se3_chordal,
                   t_sampler, r_sampler,
                   translation_noise_sigma, rotation_noise_sigma);
      continue;
    }

    std::cerr << "unrecognized edge type: " << Factory::instance()->tag(e) << std::endl;
  }

  std::cerr << std::endl;


  std::cerr << "saving noisy graph in : " << output_filename << std::endl;
  optimizer.save(output_filename.c_str());

  
  return 0;
}


void addMatchableNoise(EdgeSE3Matchable* edge_,
                       GaussianSampler<Vector3, Matrix3>& p_sampler_,
                       GaussianSampler<Vector2, Matrix2>& n_sampler_,
                       const Matrix3& point_sigma_,
                       const Matrix2& normal_sigma_) {
  //ia here gt is just the measurement since there is no noise encoded here yet
  const Matchable Z_gt = edge_->measurement();

  Vector5 noise_pertubation = Vector5::Zero();
  noise_pertubation.head(3) = p_sampler_.generateSample();
  noise_pertubation.tail(2) = n_sampler_.generateSample();

  //ia TODO check this information matrix if it is good or not
  Matchable noisy_matchable = Z_gt.applyMinimalPert(noise_pertubation);
  Matrix7 noisy_information = edge_->information();

  VertexMatchable* vm = dynamic_cast<VertexMatchable*>(edge_->vertices()[1]);
  // std::cerr << "vertex type:" << vm->estimate().type() << std::endl;
  // std::cerr << "measurement type:" << edge_->measurement().type() << std::endl;

  // std::cerr << "information\n" << edge_->information() << std::endl;
  noisy_information.block<3,3>(0,0) *=  point_sigma_.inverse();
  noisy_information.block<2,2>(4,4) *=  normal_sigma_.inverse();

  edge_->setMeasurement(noisy_matchable);
  edge_->setInformation(noisy_information);
      
  // std::cerr << "new information\n" << noisy_information << std::endl;
  // std::cin.get();
}

void addPoseNoise(EdgeSE3Chord* edge_,
                  GaussianSampler<Vector3, Matrix3>& t_sampler_,
                  GaussianSampler<Vector3, Matrix3>& r_sampler_,
                  const Matrix3& translational_sigma_,
                  const Matrix3& /*rotational_sigma_*/) {
  //ia here gt is just the measurement since there is no noise encoded here yet
  const Isometry3& z_gt =  edge_->measurement();
      
  Vector3 t_noise = t_sampler_.generateSample();
  Vector3 r_noise = r_sampler_.generateSample();
      
  Isometry3 pose_noise = Isometry3::Identity();
  pose_noise.linear() = internal::fromEuler(r_noise);
  pose_noise.translation() = t_noise;
  Isometry3 noisy_meas = Isometry3::Identity();
  noisy_meas = z_gt * pose_noise;

  Matrix12 noisy_information = Matrix12::Identity();
  //ia the information matrix is a mess,
  //ia it should be sampled in the chordal distance error space
  // noisy_information.block<3,3>(0,0) =
  //   _params.translation_noise_stats.asDiagonal().inverse();
  // noisy_information.block<3,3>(0,0) =
  //   _params.rotation_noise_stats.asDiagonal().inverse();
  noisy_information.block<9,9>(0,0) *= 1000;
  noisy_information.block<3,3>(9,9) =
    translational_sigma_.inverse();

  edge_->setMeasurement(noisy_meas);
  edge_->setInformation(noisy_information);
}
