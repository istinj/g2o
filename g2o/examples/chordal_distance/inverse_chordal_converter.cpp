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

//! @brief converts a graph with standard EDGE_SE3:CHORD into chordal EDGE_SE3:QUAT ones;

using namespace std;
using namespace g2o;

//ia useful typedefs
typedef SigmaPoint<Vector6> SigmaPoint6;
typedef SigmaPoint<Vector12> SigmaPoint12;

typedef std::vector<SigmaPoint<Vector6>,  Eigen::aligned_allocator<SigmaPoint<Vector6> > > SigmaPoint6Vector;
typedef std::vector<SigmaPoint<Vector12>, Eigen::aligned_allocator<SigmaPoint<Vector12> > > SigmaPoint12Vector;

// @brief converts a 12D <mean/cov> pdf into to standard 6D one
Matrix6 _remapBackInformationMatrix(const Vector12& src_mean_,
                                    const Matrix12& src_omega_);

int main(int argc, char *argv[]) {
  // factory setup
  VertexSE3* v_standard = new VertexSE3();
  VertexSE3Chord* v_chord = new VertexSE3Chord();
  EdgeSE3* e_standard = new EdgeSE3();
  EdgeSE3Chord* e_chord = new EdgeSE3Chord();

  // get the tags
  std::string standard_vertex_tag = Factory::instance()->tag(v_standard);
  std::string standard_edge_tag = Factory::instance()->tag(e_standard);

  // arguments of the executable
  std::string outFilename, inputFilename;
  g2o::CommandArgs arg;
  arg.param("o", outFilename, "", "output of the conversion");
  arg.paramLeftOver("graph-input", inputFilename, "", "graph file which will be processed");
  arg.parseArgs(argc, argv);


  // loading the graph
  SparseOptimizer optimizer;
  cerr << "opening the file : " << inputFilename << endl;
  std::ifstream ifs(inputFilename.c_str());
  optimizer.load(ifs);

  // getting src_vertices and src_edges from the graph
  const HyperGraph::VertexIDMap& src_vertices = optimizer.vertices();
  const HyperGraph::EdgeSet& src_edges = optimizer.edges();

  const int num_vertices = src_vertices.size();
  const int num_edges = src_edges.size();

  cerr << "Loaded " << CL_GREEN(num_vertices) << " vertexes" << endl;
  cerr << "Loaded " << CL_GREEN(num_edges) << " src_edges" << endl;

  // new src_edges
  HyperGraph::EdgeSet standard_edges;
  
  // convert the src_edges
  std::cerr << "convert the src_edges from chordal to standard quaternions" << std::endl;
  for (HyperGraph::Edge* eptr : src_edges) {
    EdgeSE3Chord* e = dynamic_cast<EdgeSE3Chord*>(eptr);
    const Isometry3& Z = e->measurement();
    const Matrix12& omega = e->information();

    // convert things
    Vector12 z = internal::toFlatten(Z);
    Matrix6 standard_omega = _remapBackInformationMatrix(z, omega);
    
    EdgeSE3* standard_edge = new EdgeSE3();
    standard_edge->setMeasurement(Z);
    standard_edge->setInformation(standard_omega);
    standard_edge->vertices() = eptr->vertices(); // this is a shit but it works
    standard_edges.insert(standard_edge);
    std::cerr << "x";
  }
  std::cerr << std::endl;

  cerr << "converted " << standard_edges.size() << " src_edges"<< endl;
  
  // write the src_vertices - which remain the same
  std::ofstream file_output_stream;
  if (outFilename != "-") {
    cerr << "Writing into " << outFilename << endl;
    file_output_stream.open(outFilename.c_str());
  } else {
    cerr << "writing to stdout" << endl;
  }

  std::ostream& fout = outFilename != "-" ? file_output_stream : cout;

  for (std::pair<const int, g2o::HyperGraph::Vertex*> vit : src_vertices) {
    VertexSE3Chord* v = dynamic_cast<VertexSE3Chord*>(vit.second);
    fout << standard_vertex_tag << " " << v->id() << " ";
    v->write(fout);
    fout << endl;
    if (v->fixed()) {
      fout << "FIX " << v->id() << endl;
    }
  }

  for (HyperGraph::Edge* eptr : standard_edges) {
    EdgeSE3* e = dynamic_cast<EdgeSE3*>(eptr);
    fout << standard_edge_tag << " " << e->vertex(0)->id() << " " << e->vertex(1)->id() << " ";
    e->write(fout);
    fout << endl;
  }

  cerr << "done!" << endl;

  // checkout things
  file_output_stream.close();

  for (HyperGraph::Edge* e : standard_edges) {
    delete e;
  }
  
  delete v_standard;
  delete v_chord;
  delete e_standard;
  delete e_chord;
}  


Matrix6 _remapBackInformationMatrix(const Vector12& src_mean_,
                                    const Matrix12& src_omega_) {

  const Matrix12 src_sigma = src_omega_.inverse();

  Vector6 remapped_mean = Vector6::Zero();
  Matrix6 remapped_sigma = Matrix6::Zero();
  Matrix6 remapped_omega = Matrix6::Zero();
  
  SigmaPoint12Vector sigma_points_12D;
  SigmaPoint6Vector sigma_points_6D;

  sampleUnscented(sigma_points_12D, src_mean_, src_sigma);

  int k = 1;
  for (int i = 0; i < sigma_points_12D[0]._sample.size(); ++i) {
    int sample_plus_idx = k++;
    int sample_minus_idx = k++;

    const SigmaPoint12& sigma_point12_plus = sigma_points_12D[sample_plus_idx];
    const SigmaPoint12& sigma_point12_minus = sigma_points_12D[sample_minus_idx];

    const Vector12& sample_12D_plus = sigma_point12_plus._sample;
    const Vector12& sample_12D_minus = sigma_point12_minus._sample;

    Isometry3 T_plus = internal::fromFlatten(sample_12D_plus, true);
    Isometry3 T_minus = internal::fromFlatten(sample_12D_minus, true);

    Vector6 sample_6D_plus = internal::toVectorMQT(T_plus);
    Vector6 sample_6D_minus = internal::toVectorMQT(T_minus);

    SigmaPoint6 point_6D_plus(sample_6D_plus, 
      sigma_point12_plus._wi, 
      sigma_point12_plus._wp);
    SigmaPoint6 point_6D_minus(sample_6D_minus, 
      sigma_point12_minus._wi, 
      sigma_point12_minus._wp);
    sigma_points_6D.push_back(point_6D_plus);
    sigma_points_6D.push_back(point_6D_minus);
  }

  reconstructGaussian(remapped_mean, remapped_sigma, sigma_points_6D);
  remapped_omega = remapped_sigma.inverse();
  return remapped_omega;
}


