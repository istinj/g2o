#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <cmath>

//ia include types
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"

//ia include g2o core stuff
#include "g2o/stuff/sampler.h"
#include "g2o/stuff/command_args.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/factory.h"

#include <Eigen/Core>
#include <Eigen/StdVector>

//! brief adds noise to an ALREADY OPTIMIZED graph
using namespace std;
using namespace g2o;

typedef Eigen::Matrix<double, 6, 6> Matrix6;

int main(int argc, char *argv[]) {
  cerr << "graph must be in a OPMIMUM STATE" << endl << endl;
  //! stuff required for the factory
  VertexSE3* vv = new VertexSE3();
  EdgeSE3* ee = new EdgeSE3();

  bool randomSeed;
  std::vector<double> noiseTranslation;
  std::vector<double> noiseRotation;
  std::string outFilename;
  std::string inputFilename;

  g2o::CommandArgs arg;
  arg.param("o", outFilename, "", "output final version of the graph");
  arg.param("noiseTranslation", noiseTranslation, std::vector<double>(), "set the noise level for the translation, separated by semicolons without spaces e.g: \"0.1;0.1;0.1\"");
  arg.param("noiseRotation", noiseRotation, std::vector<double>(), "set the noise level for the rotation, separated by semicolons without spaces e.g: \"0.001;0.001;0.001\"");
  arg.param("randomSeed", randomSeed, false, "use a randomized seed for generating the sphere");
  arg.paramLeftOver("graph-input", inputFilename, "", "graph file which will be processed");
  arg.parseArgs(argc, argv);

  if (noiseTranslation.size() == 0) {
    cerr << "using default noise for the translation" << endl;
    noiseTranslation.push_back(0.01);
    noiseTranslation.push_back(0.01);
    noiseTranslation.push_back(0.01);
  }
  cerr << "Noise for the translation:";
  for (size_t i = 0; i < noiseTranslation.size(); ++i)
    cerr << " " << noiseTranslation[i];
  cerr << endl;
  if (noiseRotation.size() == 0) {
    cerr << "using default noise for the rotation" << endl;
    noiseRotation.push_back(0.005);
    noiseRotation.push_back(0.005);
    noiseRotation.push_back(0.005);
  }
  cerr << "Noise for the rotation:";
  for (size_t i = 0; i < noiseRotation.size(); ++i)
    cerr << " " << noiseRotation[i];
  cerr << endl;

  //! compute std_dev
  Matrix3 transNoise = Matrix3::Zero();
  for (int i = 0; i < 3; ++i)
    transNoise(i, i) = std::pow(noiseTranslation[i], 2);

  Matrix3 rotNoise = Matrix3::Zero();
  for (int i = 0; i < 3; ++i)
    rotNoise(i, i) = std::pow(noiseRotation[i], 2);

  //! generate information matrix according to the noise
  Matrix6 information = Matrix6::Zero();
  information.block<3,3>(0,0) = transNoise.inverse();
  information.block<3,3>(3,3) = rotNoise.inverse();

  //! required to easily operate on the graph
  g2o::SparseOptimizer optimizer;

  //! load the graph
  cerr << "opening file : " << inputFilename << endl;
  std::ifstream ifs(inputFilename.c_str());
  optimizer.load(ifs);

  const HyperGraph::VertexIDMap& vertices = optimizer.vertices();
  const HyperGraph::EdgeSet& edges = optimizer.edges();

  int optimizer_num_v = vertices.size();
  int optimizer_num_e = edges.size();
  cerr << "Loaded " << optimizer_num_v << " vertices" << endl;
  cerr << "Loaded " << optimizer_num_e << " edges" << endl;

  //! noise generator
  GaussianSampler<Vector3, Matrix3> transSampler;
  transSampler.setDistribution(transNoise);
  GaussianSampler<Vector3, Matrix3> rotSampler;
  rotSampler.setDistribution(rotNoise);

  if (randomSeed) {
    std::random_device r;
    std::seed_seq seedSeq{r(), r(), r(), r(), r()};
    std::vector<int> seeds(2);
    seedSeq.generate(seeds.begin(), seeds.end());
    cerr << "using seeds:";
    for (size_t i = 0; i < seeds.size(); ++i)
      cerr << " " << seeds[i];
    cerr << endl;
    transSampler.seed(seeds[0]);
    rotSampler.seed(seeds[1]);
  }

  //! NOISE on the measurements
  for (HyperGraph::Edge* eptr : edges) {
    EdgeSE3* e = dynamic_cast<EdgeSE3*>(eptr);
    
    //! estimate gt
    VertexSE3* v_from = dynamic_cast<VertexSE3*>(e->vertices()[0]);
    VertexSE3* v_to = dynamic_cast<VertexSE3*>(e->vertices()[1]);

    const Isometry3& T_from = v_from->estimate();
    const Isometry3& T_to = v_to->estimate();

    Isometry3 Z_gt = T_from.inverse() * T_to;
    Eigen::Quaterniond gt_quat(Z_gt.linear());
    Vector3 gt_trans(Z_gt.translation());

    //! sample
    Vector3 noise_quatXYZ = rotSampler.generateSample();
    double noise_qw = 1.0 - noise_quatXYZ.norm();
    if (noise_qw < 0) {
      noise_qw = 0.;
      cerr << "x";
    }

    Eigen::Quaterniond rot_noise(noise_qw, noise_quatXYZ.x(), noise_quatXYZ.y(), noise_quatXYZ.z());
    rot_noise.normalize();
    Vector3 trans_noise = transSampler.generateSample();

    Eigen::Quaterniond noisy_quat = gt_quat * rot_noise;
    Isometry3 noisy_meas = Isometry3::Identity();
    noisy_meas.linear() = noisy_quat.matrix();
    noisy_meas.translation() = gt_trans + trans_noise;

    e->setMeasurement(noisy_meas);
    e->setInformation(information);
  }

  // write output
  std::ofstream fileOutputStream;
  if (outFilename != "-") {
    cerr << "Writing into " << outFilename << endl;
    fileOutputStream.open(outFilename.c_str());
  } else {
    cerr << "writing to stdout" << endl;
  }

  std::string vertexTag = Factory::instance()->tag(vv);
  std::string edgeTag = Factory::instance()->tag(ee);

  std::ostream& fout = outFilename != "-" ? fileOutputStream : cout;
  for (std::pair<const int, g2o::HyperGraph::Vertex*> vit : vertices) {
    VertexSE3* v = dynamic_cast<VertexSE3*>(vit.second);
    fout << vertexTag << " " << v->id() << " ";
    v->write(fout);
    fout << endl;
    if (v->fixed()) {
      fout << "FIX " << v->id() << endl;
    }
  }

  for (HyperGraph::Edge* eptr : edges) {
    EdgeSE3* e = dynamic_cast<EdgeSE3*>(eptr);
    VertexSE3* from = dynamic_cast<VertexSE3*>(e->vertex(0));
    VertexSE3* to = dynamic_cast<VertexSE3*>(e->vertex(1));
    fout << edgeTag << " " << from->id() << " " << to->id() << " ";
    e->write(fout);
    fout << endl;
  }

  delete ee; delete vv;

  return 0;
}

