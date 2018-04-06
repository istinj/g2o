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

using namespace g2o;
using namespace std;

int main(int argc, char **argv) {

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
  optimizer.setVerbose(verbose);

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

  std::cerr << "loaded " << optimizer.vertices().size() << " vertices" << std::endl;
  std::cerr << "loaded " << optimizer.edges().size() << " edges" << std::endl;


  //ia load the graph
  return 0;
}
