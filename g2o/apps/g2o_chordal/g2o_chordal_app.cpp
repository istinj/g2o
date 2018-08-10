// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <signal.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cassert>

#include "g2o/apps/g2o_cli/dl_wrapper.h"
#include "g2o/apps/g2o_cli/output_helper.h"
#include "g2o/apps/g2o_cli/g2o_common.h"

#include "g2o/config.h"
#include "g2o/core/estimate_propagator.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/hyper_dijkstra.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/batch_stats.h"
#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_factory.h"
#include "g2o/core/optimization_algorithm.h"
#include "g2o/core/sparse_optimizer_terminate_action.h"

#include "g2o/stuff/macros.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"

static bool has_to_stop=false;

using namespace std;
using namespace g2o;

void sigquitHandler(int sig) {
  if (sig == SIGINT) {
    has_to_stop = 1;
    static int cnt = 0;
    if (cnt++ == 2) {
      std::cerr << __PRETTY_FUNCTION__ << " forcing exit" << std::endl;
      exit(1);
    }
  }
}

int main(int argc, char** argv) {
  OptimizableGraph::initMultiThreading();

  // registering all the types from the libraries
  DlWrapper dl_types_wrapper;
  loadStandardTypes(dl_types_wrapper, argc, argv);
  // register all the solvers
  DlWrapper dl_solver_wrapper;
  loadStandardSolver(dl_solver_wrapper, argc, argv);

  // Command line parsing
  bool verbose;
  bool initial_guess;
  bool initial_guess_odometry;

  string initial_guess_type("no-guess");

  bool list_types_flag;
  bool list_solvers_flag;
  bool list_kernels_flag;

  int max_iterations;
  double kernel_width;

  string output_filename;
  string input_filename;
  string stats_filename;
  string summary_filaname;
  string kernel_type;
  string solver_type;

  bool optimization_has_kernel = false;

  g2o::CommandArgs arg;
  arg.param("v", verbose, false, "verbose output of the optimization process");
  arg.param("listTypes", list_types_flag, false, "list the registered types");
  arg.param("listRobustKernels", list_kernels_flag, false, "list the registered robust kernels");
  arg.param("listSolvers", list_solvers_flag, false, "list the available solvers");
  arg.param("i", max_iterations, 10, "perform n iterations, if negative consider the gain");
  arg.param("o", output_filename, "", "output final version of the graph");
  arg.param("guess", initial_guess, false, "initial guess based on spanning tree");
  arg.param("guessOdometry", initial_guess_odometry, false, "initial guess based on odometry");
  arg.param("robustKernel", kernel_type, "", "use this robust error function");
  arg.param("robustKernelWidth", kernel_width, -1., "width for the robust Kernel (only if robust_kernel_type)");
  arg.param("solver", solver_type, "gn_fix6_3_cholmod", "specify which solver to use underneat\n\t {gn_var, lm_fix3_2, gn_fix6_3, lm_fix7_3}");
  arg.param("stats", stats_filename, "", "specify a file for the stats");
  arg.param("summary", summary_filaname, "", "summary of the optimization - for tables :)");
  arg.paramLeftOver("graph-input", input_filename, "", "file to be processed");
  arg.parseArgs(argc, argv);

  //ia factory things
  OptimizationAlgorithmFactory* solver_factory = OptimizationAlgorithmFactory::instance();
  if (list_solvers_flag) {
    solver_factory->listSolvers(std::cerr);
    return 0;
  }

  if (list_types_flag) {
    Factory::instance()->printRegisteredTypes(std::cerr, true);
    return 0;
  }

  if (list_kernels_flag) {
    std::vector<std::string> available_kernels;
    RobustKernelFactory::instance()->fillKnownKernels(available_kernels);
    std::cerr << "Robust Kernels:" << std::endl;
    for (size_t i = 0; i < available_kernels.size(); ++i) {
      std::cerr << available_kernels[i] << std::endl;
    }
    return 0;
  }

  if (verbose) {
    std::cerr << "# Used Compiler: " << G2O_CXX_COMPILER << std::endl;
  }

  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(verbose);
  optimizer.setForceStopFlag(&has_to_stop);

  //ia checking that you are not dumb
  if (input_filename == "") {
    throw std::runtime_error("no input graph, exit");
  }



  //ia create a solver for the chordal optimizer
  OptimizationAlgorithmProperty solver_property;
  optimizer.setAlgorithm(solver_factory->construct(solver_type, solver_property));
  if (! optimizer.solver()) {
    throw std::runtime_error("error while allocating the solver, exit");
  }

  //ia read the damn chordal file
  std::cerr << "opening file: " << input_filename << std::endl;
  std::ifstream input_stream(input_filename.c_str());
  if (!input_stream)
    throw std::runtime_error("failed to open the selected file, exit");
  optimizer.load(input_stream);
  if (optimizer.vertices().size() == 0)
    throw std::runtime_error("graph contains no vertices, exit");
  const size_t optimizer_num_v = optimizer.vertices().size();
  const size_t optimizer_num_e = optimizer.edges().size();
  std::cerr << "loaded " << optimizer_num_v << " vertices" << std::endl;
  std::cerr << "loaded " << optimizer_num_e << " edges" << std::endl;

  //ia check that the solver is compatible
  std::set<int> vertices_dimensions = optimizer.dimensions();
  if (!optimizer.isSolverSuitable(solver_property, vertices_dimensions)) {
    throw std::runtime_error("solver is not suitable for optimizing the chordal graph, exit");
  }

  //ia we assume that the gauge is already specified, to avoid that g2o fixes different vertices
  //ia between the two graphs
  if (optimizer.gaugeFreedom())
    throw std::runtime_error("selected graph has no fixed vertices, exit");


  //ia robust kernels, if present
  if (kernel_type.size() > 0) {
    optimization_has_kernel = true;
    std::cerr << "using kernel " << kernel_type << std::endl;
    AbstractRobustKernelCreator* kernel_factory = RobustKernelFactory::instance()->creator(kernel_type);
    if (!kernel_factory)
      throw std::runtime_error("unknown kernel type type, call -listKernels man");

    SparseOptimizer::EdgeSet::iterator edge_it = optimizer.edges().begin();
    SparseOptimizer::EdgeSet::iterator edges_end = optimizer.edges().end();
    while (edge_it != edges_end) {
      SparseOptimizer::Edge* e = dynamic_cast<SparseOptimizer::Edge*>(*edge_it);
      e->setRobustKernel(kernel_factory->construct());
      if (kernel_width > 0)
        e->robustKernel()->setDelta(kernel_width);
      ++edge_it;
    }
  }

  //ia final tweaks
  signal(SIGINT, sigquitHandler);
  optimizer.setComputeBatchStatistics(true);

  //ia initialize optimization
  double initial_chi2 = -1.0;
  double final_chi2 = -1.0;
  optimizer.initializeOptimization();

  //ia initial guess
  if (initial_guess && initial_guess_odometry)
    throw std::runtime_error("both initialization techniques selected, exit");

  if (initial_guess) {
    std::cerr << "computing initial guess from spanning tree" << std::endl;
    initial_guess_type = "spanning";
    optimizer.computeInitialGuess();
  }

  if (initial_guess_odometry) {
    std::cerr << "computing initial guess from odometry" << std::endl;
    initial_guess_type = "odometry";
    EstimatePropagatorCostOdometry chordal_odometry_cost_function(&optimizer);
    optimizer.computeInitialGuess(chordal_odometry_cost_function);
  }

  //ia get the damn initial chi2
  optimizer.computeActiveErrors();
  if (optimization_has_kernel)
    initial_chi2 = optimizer.activeRobustChi2();
  else
    initial_chi2 = optimizer.activeChi2();
  std::cerr << "initial chi2: " << FIXED(initial_chi2) << std::endl;

  //ia do the damn job
  int optimization_result = optimizer.optimize(max_iterations);
  if (max_iterations > 0 && optimization_result == OptimizationAlgorithm::Fail)
    std::cerr << "Cholesky failed, results may be invalid" << std::endl;

  optimizer.computeActiveErrors();
  if (optimization_has_kernel)
    final_chi2 = optimizer.activeRobustChi2();
  else
    final_chi2 = optimizer.activeChi2();


  //ia shit out the statitics
  if (stats_filename != "") {
    std::cerr << "writing stats to file: " << stats_filename << " ... ";
    ofstream stats_stream(stats_filename.c_str());

    //ia write firstly the damn initial chi2
    G2OBatchStatistics initial_stats;
    initial_stats.iteration = -1;
    initial_stats.chi2 = initial_chi2;

    BatchStatisticsContainer complete_bsc = optimizer.batchStatistics();
    complete_bsc.insert(complete_bsc.begin(), initial_stats);

    for (size_t i = 0; i < complete_bsc.size(); i++) {
      const G2OBatchStatistics& s = complete_bsc[i];
      if (s.choleskyNNZ == 0 && i != 0)
        continue;
      stats_stream << s << std::endl;
    }
    std::cerr << "done." << std::endl;
    //ia porco dio close the stats
    stats_stream.close();
  }


  //ia shit out a summary useful for latex tables
  if (summary_filaname != "") {
    std::cerr << "writing summary to file: " << summary_filaname << " ... ";
    PropertyMap summary;
    summary.makeProperty<StringProperty>("filename", input_filename);
    summary.makeProperty<IntProperty>("n_vertices", optimizer.vertices().size());
    summary.makeProperty<IntProperty>("n_edges", optimizer.edges().size());
    summary.makeProperty<StringProperty>("initial_guess_type", initial_guess_type);
    summary.makeProperty<IntProperty>("num_iterations", max_iterations);
    summary.makeProperty<StringProperty>("solver_type", solver_type);
    summary.makeProperty<DoubleProperty>("initial_chi", initial_chi2);
    summary.makeProperty<DoubleProperty>("final_chi", final_chi2);

    std::ofstream summary_stream(summary_filaname.c_str());
    summary.writeToCSV(summary_stream);
    summary_stream.close();
    std::cerr << "done." << std::endl;
  }


  //ia save the output graph
  if (output_filename.size() > 0) {
    std::cerr << "saving final graph to file: " << output_filename << " ... ";
    optimizer.save(output_filename.c_str());
    std::cerr << "done." << std::endl;
  }


  return 0;
}
