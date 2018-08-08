#include <iostream>
#include <string>
#include <map>

//ia include fancy colors
#include "g2o/stuff/color_macros.h"

//ia include types
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"

//ia include g2o core stuff
#include "g2o/stuff/command_args.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"

//ia include unscented
#include "g2o/stuff/unscented.h"

#include <Eigen/Core>
#include <Eigen/StdVector>

//! @brief converts a graph with standard EDGE_SE3:QUAT into chordal EDGE_SE3:CHORD ones;

using namespace std;
using namespace g2o;

//ia useful typedefs
typedef std::pair<int, int> IntPair;
typedef SigmaPoint<Vector6> SigmaPoint6;
typedef SigmaPoint<Vector12> SigmaPoint12;

typedef std::vector<SigmaPoint<Vector6>,  Eigen::aligned_allocator<SigmaPoint<Vector6> > > SigmaPoint6Vector;
typedef std::vector<SigmaPoint<Vector12>, Eigen::aligned_allocator<SigmaPoint<Vector12> > > SigmaPoint12Vector;

// @brief omega remapper
Matrix12 _remapInformationMatrix(const Vector6& src_mean_,
                                 const Matrix6& src_omega_,
                                 const uint8_t conditioning_type_,
                                 const double& thresh_);

Matrix12 _reconditionateSigma(const Matrix12& src_sigma_,
                              const uint8_t type_,
                              const double& threshold_);


int main(int argc, char *argv[]) {
  // factory setup
  VertexSE3* v = new VertexSE3();
  VertexSE3Chord* v_chord = new VertexSE3Chord();
  EdgeSE3* e_standard = new EdgeSE3();
  EdgeSE3Chord* e_chord = new EdgeSE3Chord();

  std::string chord_vertex_tag = Factory::instance()->tag(v_chord);
  std::string chord_edge_tag = Factory::instance()->tag(e_chord);

  // arguments of the executable
  std::string outFilename, inputFilename;
  double omegaThreshold;
  int conditioningType;
  g2o::CommandArgs arg;
  arg.param("o", outFilename, "", "output of the conversion");
  arg.param("omegaTresh", omegaThreshold, 1e-1, "threshold used to remap the information matrix of the edges");
  arg.param("condType", conditioningType, 0, "type of conditioning: 0->SVD based, 1->add stuff on diagonal");
  arg.paramLeftOver("graph-input", inputFilename, "", "graph file which will be processed");
  arg.parseArgs(argc, argv);


  // loading the graph
  SparseOptimizer optimizer;
  cerr << "opening the file : " << inputFilename << endl;
  std::ifstream ifs(inputFilename.c_str());
  optimizer.load(ifs);

  // getting vertices and edges from the graph
  const HyperGraph::VertexIDMap& vertices = optimizer.vertices();
  const HyperGraph::EdgeSet& edges = optimizer.edges();

  const int num_vertices = vertices.size();
  const int num_edges = edges.size();

  cerr << "Loaded " << CL_GREEN(num_vertices) << " vertexes" << endl;
  cerr << "Loaded " << CL_GREEN(num_edges) << " edges" << endl;

  // new edges
  HyperGraph::EdgeSet chordal_edges;
  
  // convert the edges
  size_t cnt = 0;
  std::cerr << "conversion started" << std::endl;
  for (HyperGraph::Edge* eptr : edges) {
    float percentage = std::ceil((float)cnt++ / (float)edges.size() * 100);
    if ((int)percentage % 5 == 0)
      std::cerr << "\rcompleted " << percentage << "%" << std::flush;
    
    EdgeSE3* e = dynamic_cast<EdgeSE3*>(eptr);
    const Isometry3& Z = e->measurement();
    const Matrix6& omega = e->information();

    // convert things
    Vector6 z = internal::toVectorMQT(Z);
    Matrix12 omega_chord = _remapInformationMatrix(z, omega, conditioningType, omegaThreshold);
    
    EdgeSE3Chord* chord_edge = new EdgeSE3Chord();
    chord_edge->setMeasurement(Z);
    chord_edge->setInformation(omega_chord);
    chord_edge->vertices() = eptr->vertices();
    chordal_edges.insert(chord_edge);
  }
  std::cerr << "\n";

  cerr << "converted " << chordal_edges.size() << " edges"<< endl;
  
  // write the vertices - which remain the same
  std::ofstream file_output_stream;
  if (outFilename != "-") {
    cerr << "Writing into " << outFilename << endl;
    file_output_stream.open(outFilename.c_str());
  } else {
    cerr << "writing to stdout" << endl;
  }
  
  std::ostream& fout = outFilename != "-" ? file_output_stream : cout;

  for (std::pair<const int, g2o::HyperGraph::Vertex*> vit : vertices) {
    VertexSE3* v = dynamic_cast<VertexSE3*>(vit.second);
    fout << chord_vertex_tag << " " << v->id() << " ";
    v->write(fout);
    fout << endl;
    if (v->fixed()) {
      fout << "FIX " << v->id() << endl;
    }
  }

  for (HyperGraph::Edge* eptr : chordal_edges) {
    EdgeSE3Chord* e = dynamic_cast<EdgeSE3Chord*>(eptr);
    fout << chord_edge_tag << " " << e->vertex(0)->id() << " " << e->vertex(1)->id() << " ";
    e->write(fout);
    fout << endl;
  }

  cerr << "done!" << endl;

  // checkout things
  file_output_stream.close();

  for (HyperGraph::Edge* e : chordal_edges) {
    delete e;
  }
  
  delete v;
  delete v_chord;
  delete e_standard;
  delete e_chord;
}  



Matrix12 _remapInformationMatrix(const Vector6& src_mean_,
                                 const Matrix6& src_omega_,
                                 const uint8_t reconditioning_type_,
                                 const double& threshold_) {
  //ia computing sigma_12D
  Matrix6 src_sigma = src_omega_.inverse();
  Isometry3 T_mean = internal::fromVectorMQT(src_mean_);

  Vector12 remapped_mean = Vector12::Zero();
  Matrix12 remapped_sigma = Matrix12::Zero();

  SigmaPoint6Vector sigma_points_6D;
  SigmaPoint12Vector sigma_points_12D;

  Vector6 zero_mean = Vector6::Zero();
  if (!sampleUnscented(sigma_points_6D, zero_mean, src_sigma))
    throw std::runtime_error("bad things happened while sampling");

  int k = 1;
  for (int i = 0; i < src_mean_.size(); ++i) {
    int sample_plus_idx = k++;
    int sample_minus_idx = k++;

    const Vector6& sample_6D_plus = sigma_points_6D[sample_plus_idx]._sample;
    const Vector6& sample_6D_minus = sigma_points_6D[sample_minus_idx]._sample;

    Isometry3 T_plus = internal::fromVectorMQT(sample_6D_plus);
    Isometry3 T_minus = internal::fromVectorMQT(sample_6D_minus);

    Vector12 sample_12D_plus = internal::toFlatten(T_mean*T_plus);
    Vector12 sample_12D_minus = internal::toFlatten(T_mean*T_minus);

    SigmaPoint12 point_12D_plus(sample_12D_plus, sigma_points_6D[sample_plus_idx]._wi, sigma_points_6D[sample_plus_idx]._wp);
    SigmaPoint12 point_12D_minus(sample_12D_minus, sigma_points_6D[sample_minus_idx]._wi, sigma_points_6D[sample_minus_idx]._wp);
    sigma_points_12D.push_back(point_12D_plus);
    sigma_points_12D.push_back(point_12D_minus);
  }

  reconstructGaussian(remapped_mean, remapped_sigma, sigma_points_12D);
  
  //ia reconditioning the new covariance
  Matrix12 conditioned_sigma = _reconditionateSigma(remapped_sigma, reconditioning_type_, threshold_);
  Matrix12 remapped_omega = conditioned_sigma.inverse();
  // remapped_omega.block<9,9>(0,0) = Matrix9::Identity();
  // remapped_omega.block<3,3>(9,9) = Matrix3::Identity();
  return remapped_omega;
}


Matrix12 _reconditionateSigma(const Matrix12& src_sigma_,
                              const uint8_t type_,
                              const double& threshold_) {

  Matrix12 conditioned_sigma = Matrix12::Zero();

  switch (type_) {
  case 0: //ia soft conditioning
    {
      Eigen::JacobiSVD<Matrix12> svd(src_sigma_, Eigen::ComputeThinU | Eigen::ComputeThinV);
      double conditioned_eigenvalue = 1.0;
      for (int i = 0; i < 12; ++i) {
        // std::cerr << GREEN << "singular_v = " << svd.singularValues()(i,i) << std::endl << RESET;
        //! best
        if (svd.singularValues()(i,i) < threshold_) {
          conditioned_eigenvalue = svd.singularValues()(i,i) + threshold_;
        } else {
          conditioned_eigenvalue = svd.singularValues()(i,i);
        }
        conditioned_sigma.noalias() += conditioned_eigenvalue *
          svd.matrixU().col(i) * 
          svd.matrixU().col(i).transpose();
        // std::cerr << "conditioned_eigenvalue = " << conditioned_eigenvalue << std::endl;
      }
      // std::cin.get();
      break;
    }
    
  case 1: //ia mid conditioning - pump the value on the diagonal
    {
      conditioned_sigma = src_sigma_;
      conditioned_sigma.diagonal().array() += threshold_; //! Avoids rank loss (right value)
      // conditioned_sigma.diagonal().array() += 1e-10; //! Avoids rank loss (this value will restore original omegas +/-)
      break;
    }
  default:
    std::cerr << "wrong conditionig type; allowed are -> 0 [soft], 1 [mid]" << std::endl;
    break;
  }
  return conditioned_sigma;
}
