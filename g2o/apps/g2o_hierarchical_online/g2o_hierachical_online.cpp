#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cassert>
#include <sstream>

//ia include fancy colors
#include "g2o/stuff/macros.h"
#include "g2o/stuff/color_macros.h"

//ia include types
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"

//ia include g2o core stuff
#include "g2o/stuff/command_args.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/robust_kernel_factory.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/robust_kernel.h"
#include "g2o/apps/g2o_cli/dl_wrapper.h"
#include "g2o/apps/g2o_cli/output_helper.h"
#include "g2o/apps/g2o_cli/g2o_common.h"

#include "local_star.h"

using namespace g2o;
using namespace std;

int main(int argc, char **argv) {

  const std::string backbone_tag3("VERTEX_SE3:QUAT");
  const std::string backbone_tag2("VERTEX_SE2:QUAT");

  //ia parse commands
  std::string robust_kernel;
  std::string input_filename;
  std::string output_filename;
  std::string solver_type;
  double huber_width;
  bool verbose;
  bool debug;
  bool list_solvers;
  bool list_robust_kernels;

  // command line parsing
  g2o::CommandArgs arg;
  arg.param("v", verbose, false, "verbose output of the optimization process");
  arg.param("debug", debug, false, "print shit load of things for debugging");
  arg.param("robustKernel", robust_kernel, "", "use this robust error function");
  arg.param("robussolverFactorytKernelWidth", huber_width, -1., "width for the robust Kernel (only if robustKernel)");
  arg.param("huberWidth", huber_width, -1., "width for the robust Huber Kernel (only if robustKernel)");
  arg.param("o", output_filename, "", "output final version of the graph");
  arg.param("solver", solver_type, "gn_var_cholmod", "specify which solver to use");
  arg.param("listSolvers", list_solvers, false, "list the available solvers");
  arg.param("listRobustKernels", list_robust_kernels, false, "list the registered robust kernels");

  arg.paramLeftOver("graph-input", input_filename, "", "graph file which will be processed", true);

  arg.parseArgs(argc, argv);

  // ia registering all the types from the libraries
  g2o::DlWrapper dlTypesWrapper;
  g2o::loadStandardTypes(dlTypesWrapper, argc, argv);

  g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();

  //list solvers options
  //why this does not work??
  if (list_solvers) {
    solver_factory->listSolvers(std::cout);
    return 0;
  }

  //list robust kernels
  if (list_robust_kernels) {
    std::vector<std::string> kernels;
    g2o::RobustKernelFactory::instance()->fillKnownKernels(kernels);
    std::cout << "Robust Kernels:" << std::endl;
    for (size_t i = 0; i < kernels.size(); ++i) {
      std::cout << kernels[i] << std::endl;
    }
    return 0;
  }

  //create a robust kernel
  g2o::AbstractRobustKernelCreator* kernel_creator = 0;
  if (robust_kernel.size() > 0) {
    kernel_creator = g2o::RobustKernelFactory::instance()->creator(robust_kernel);
  }

  //create the optimizer and load the original graph
  g2o::SparseOptimizer optimizer;

  if (input_filename.size() == 0) {
    throw std::runtime_error("no input data specified");
  } else if (input_filename == "-") {
    std::cerr << "read input from stdin" << std::endl;
    if (!optimizer.load(cin)) {
      std::cerr << "error loading graph" << std::endl;
      return 2;
    }
  } else {
    std::cerr << "read input from " << input_filename << std::endl;
    std::ifstream ifs(input_filename.c_str());
    if (!ifs) {
      throw std::runtime_error("failed to open file");
    }
    if (!optimizer.load(ifs)) {
      throw std::runtime_error("error while loading graph");
    }
  }

  std::cerr << "loaded " << optimizer.vertices().size() << " vertices and "
            << optimizer.edges().size() << " edges"  << std::endl;


  //ia get the backbone
  const g2o::HyperGraph::EdgeSet& src_edges = optimizer.edges();
  const g2o::HyperGraph::VertexIDMap& src_vertices = optimizer.vertices();
  std::map<const int, g2o::HyperGraph::Vertex*> backbone_vertices;

  std::cerr << "reordering the backbone" << std::endl;
  g2o::HyperGraph::VertexIDMap::const_iterator v_it = src_vertices.begin();
  g2o::HyperGraph::VertexIDMap::const_iterator v_end = src_vertices.end();
  g2o::Factory* factory = g2o::Factory::instance();
  while (v_it != v_end) {
    if (factory->tag(v_it->second) == backbone_tag3 || factory->tag(v_it->second) == backbone_tag2) {
      backbone_vertices.insert(std::make_pair(v_it->first, v_it->second));
    }
    ++v_it;
  }

  std::cerr << "local stars" << std::endl;

  StarHandler sh;
  sh.setDistanceThresh(5.0);
  sh.init();
  for (std::map<const int, g2o::HyperGraph::Vertex*>::const_iterator it = backbone_vertices.begin();
      it != backbone_vertices.end(); ++it) {
    std::cerr << "vertex #" << it->second->id() << "\telement type: " << factory->tag(it->second) << std::endl;

    sh.addVertexToStar(it->second);
    std::cerr << "number of stars = " << sh.size() << std::endl;

    for (HyperGraph::Edge* e : it->second->edges()) {
      std::cerr << factory->tag(e) << std::endl;
    }

    std::cin.get();
  }




  // ia create another optimizer where we pass the nodes and the edges incrementally
  // ia to simulate online functioning for now just
  g2o::SparseOptimizer online_optimizer;
  online_optimizer.setVerbose(verbose);

  return 0;
}

