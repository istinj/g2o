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

#include "g2o/types/slam3d/types_slam3d.h"

using namespace std;
using namespace g2o;

int main(int argc, char *argv[]) {
  // registering all the types from the libraries
  DlWrapper dlTypesWrapper;
  loadStandardTypes(dlTypesWrapper, argc, argv);
  // register all the solvers
  DlWrapper dlSolverWrapper;
  loadStandardSolver(dlSolverWrapper, argc, argv);

  // Command line parsing
  bool initial_guess;
  bool initial_guess_odometry;

  bool list_types_flag;
  bool list_solvers_flag;
  bool list_kernels_flag;
  
  int max_iterations;
  double kernel_width;
  
  string output_filename;
  string chordal_input_filename;
  string kernel_type;
  string stats_filename;
  string comparison_stats_filename; 
  string geodesic_graph_filename;
  string solver_type;

  g2o::CommandArgs arg;
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
  arg.param("compareStats", comparison_stats_filename, "", "specify a file for comparison stats");
  arg.param("stats", stats_filename, "", "specify a file for the statistics");
  arg.param("otherGraph", geodesic_graph_filename, "", "standard graph to compare - geodesic error");
  arg.paramLeftOver("graph-input", chordal_input_filename, "", "chordal graph file which will be processed");
  arg.parseArgs(argc, argv);

  //ia checking that you are not dumb
  if (geodesic_graph_filename == "") {
    throw std::runtime_error("no comparison graph, exit");
  }

  if (comparison_stats_filename == "") {
    throw std::runtime_error("no comparison stats filename, exit");
  }

  if (chordal_input_filename == "") {
    throw std::runtime_error("no input man, exit");
  }

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


  // ---------------------------------------------------------------------------------- //
  // ---------------------------------------------------------------------------------- //
  // ---------------------------- chordal optimizer setup ----------------------------- //
  // ---------------------------------------------------------------------------------- //
  // ---------------------------------------------------------------------------------- //

  //ia setup chordal optimizer
  g2o::SparseOptimizer chordal_optimizer;
  chordal_optimizer.setVerbose(true);

  //ia create a solver for the chordal optimizer
  OptimizationAlgorithmProperty solver_property;
  chordal_optimizer.setAlgorithm(solver_factory->construct(solver_type, solver_property));
  if (! chordal_optimizer.solver()) {
    throw std::runtime_error("error while allocating the solver, exit");
  }

  //ia read the damn chordal file
  std::cerr << "opening file: " << chordal_input_filename << std::endl;
  std::ifstream input_stream_chordal(chordal_input_filename.c_str());
  if (!input_stream_chordal)
    throw std::runtime_error("failed to open the selected file, exit");
  chordal_optimizer.load(input_stream_chordal);
  const size_t chordal_optimizer_num_v = chordal_optimizer.vertices().size();
  const size_t chordal_optimizer_num_e = chordal_optimizer.edges().size();
  std::cerr << "loaded " << chordal_optimizer_num_v << " vertices" << std::endl;
  std::cerr << "loaded " << chordal_optimizer_num_e << " edges" << std::endl;

  //ia check that the solver is compatible
  std::set<int> vertices_dimensions = chordal_optimizer.dimensions();
  if (!chordal_optimizer.isSolverSuitable(solver_property, vertices_dimensions)) {
    throw std::runtime_error("solver is not suitable for optimizing the chordal graph, exit");
  }

  //ia we assume that the gauge is already specified, to avoid that g2o fixes different vertices
  //ia between the two graphs
  if (chordal_optimizer.gaugeFreedom())
    throw std::runtime_error("selected chordal graph has no fixed vertices, exit");


  //ia robust kernels, if present
  if (kernel_type.size() > 0) {
    std::cerr << "using kernel " << kernel_type << std::endl;
    AbstractRobustKernelCreator* kernel_factory = RobustKernelFactory::instance()->creator(kernel_type);
    if (!kernel_factory)
      throw std::runtime_error("unknown kernel type type, call -listKernels man");

    SparseOptimizer::EdgeSet::iterator edge_it = chordal_optimizer.edges().begin();
    SparseOptimizer::EdgeSet::iterator edges_end = chordal_optimizer.edges().end();
    while (edge_it != edges_end) {
      SparseOptimizer::Edge* e = dynamic_cast<SparseOptimizer::Edge*>(*edge_it);
      e->setRobustKernel(kernel_factory->construct());
      if (kernel_width > 0)
        e->robustKernel()->setDelta(kernel_width);
      ++edge_it;
    }
  }

  //ia final tweaks
  if (stats_filename!=""){
    chordal_optimizer.setComputeBatchStatistics(true);
  }

  // ---------------------------------------------------------------------------------- //
  // ---------------------------------------------------------------------------------- //
  // --------------------------- geodesic optimizer setup ----------------------------- //
  // ---------------------------------------------------------------------------------- //
  // ---------------------------------------------------------------------------------- //
  
  //ia setup geodesic optimizer
  g2o::SparseOptimizer geodesic_optimizer;
  geodesic_optimizer.setVerbose(true);

  //ia create a solver for the chordal optimizer
  OptimizationAlgorithmProperty solver_property_geodesic;
  chordal_optimizer.setAlgorithm(solver_factory->construct(solver_type, solver_property_geodesic));
  if (!geodesic_optimizer.solver()) {
    throw std::runtime_error("error while allocating the solver for the standard graph, exit");
  }

  //ia read the damn geodesic file
  std::cerr << "opening comparative file: " << geodesic_graph_filename << std::endl;
  std::ifstream input_stream_geodesic(geodesic_graph_filename.c_str());
  if (!input_stream_geodesic)
    throw std::runtime_error("failed to open the comparative file, exit");
  geodesic_optimizer.load(input_stream_geodesic);
  const size_t geodesic_optimizer_num_v = geodesic_optimizer.vertices().size();
  const size_t geodesic_optimizer_num_e = geodesic_optimizer.edges().size();
  std::cerr << "loaded " << geodesic_optimizer_num_v << " vertices" << std::endl;
  std::cerr << "loaded " << geodesic_optimizer_num_e << " edges" << std::endl;

  //ia check that the solver is compatible
  std::set<int> geodesic_vertices_dimensions = geodesic_optimizer.dimensions();
  if (!geodesic_optimizer.isSolverSuitable(solver_property, geodesic_vertices_dimensions)) {
    throw std::runtime_error("solver is not suitable for optimizing the standard graph, exit");
  }

  //ia we assume that the gauge is already specified, to avoid that g2o fixes different vertices
  //ia between the two graphs
  if (geodesic_optimizer.gaugeFreedom())
    throw std::runtime_error("selected chordal graph has no fixed vertices, exit");


  //ia robust kernels, if present
  if (kernel_type.size() > 0) {
    std::cerr << "kernelize the standard graph" << std::endl;
    AbstractRobustKernelCreator* kernel_factory = RobustKernelFactory::instance()->creator(kernel_type);
    if (!kernel_factory)
      throw std::runtime_error("unknown kernel type type, call -listKernels man");

    SparseOptimizer::EdgeSet::iterator edge_it = geodesic_optimizer.edges().begin();
    SparseOptimizer::EdgeSet::iterator edges_end = geodesic_optimizer.edges().end();
    while (edge_it != edges_end) {
      SparseOptimizer::Edge* e = dynamic_cast<SparseOptimizer::Edge*>(*edge_it);
      e->setRobustKernel(kernel_factory->construct());
      if (kernel_width > 0)
        e->robustKernel()->setDelta(kernel_width);
      ++edge_it;
    }
  }

  //ia final tweaks
  if (stats_filename!=""){
    geodesic_optimizer.setComputeBatchStatistics(true);
  }



  // ---------------------------------------------------------------------------------- //
  // ---------------------------------------------------------------------------------- //
  // --------------------------- start the damn computation --------------------------- //
  // ---------------------------------------------------------------------------------- //
  // ---------------------------------------------------------------------------------- //
  std::cerr << "comparison statistic file: " << comparison_stats_filename << std::endl;
  std::cerr << "stats legend: <iteration>; <chordal chi2>; <reprojected geodesic chi2>" << std::endl;
  std::ofstream comp_stats_stream(comparison_stats_filename);

  //ia initialize optimization
  chordal_optimizer.initializeOptimization();
  chordal_optimizer.computeActiveErrors();
  const double chordal_load_chi2 = chordal_optimizer.chi2(); 

  geodesic_optimizer.initializeOptimization();
  geodesic_optimizer.computeActiveErrors();
  const double geodesic_load_chi2 = geodesic_optimizer.chi2();
  
  std::cerr << "chordal graph load chi2  : " << FIXED(chordal_load_chi2) << std::endl;
  std::cerr << "geodesic graph load chi2 : " << FIXED(geodesic_load_chi2) << std::endl;

  if (initial_guess) {
    std::cerr << "computing initial guess from spanning tree" << std::endl;
    chordal_optimizer.computeInitialGuess();
    geodesic_optimizer.computeInitialGuess();
  } else if (initial_guess_odometry) {
    std::cerr << "computing initial guess from odometry" << std::endl;
    EstimatePropagatorCostOdometry chordal_odometry_cost_function(&chordal_optimizer);
    EstimatePropagatorCostOdometry geodesic_odometry_cost_function(&geodesic_optimizer);
  }

  for (int i = 0; i < max_iterations; ++i) {
    int optimization_result = chordal_optimizer.optimize(1); //ia one step at time
    if (optimization_result == OptimizationAlgorithm::Fail) {
      std::cerr << "cholesky failed, stop" << std::endl;
      break;
    }

    //ia copy the damn vertices in the geodesic graph and compute the chi2
    HyperGraph::VertexIDMap::iterator v_it = chordal_optimizer.vertices().begin();
    HyperGraph::VertexIDMap::iterator v_end = chordal_optimizer.vertices().end();
    while (v_it != v_end) {
      //ia take the cordal vertex
      VertexSE3Chord* chordal_v = dynamic_cast<VertexSE3Chord*>(v_it->second);
      //ia take the corresponding geodesic vertex
      VertexSE3* geodesic_v = dynamic_cast<VertexSE3*>(geodesic_optimizer.vertex(v_it->first));

      if (!chordal_v || !geodesic_v)
        throw std::runtime_error("dynamic cast failed, exit");

      //ia copy the damn estimate
      geodesic_v->setEstimate(chordal_v->estimate());
      
      ++v_it;
    }

    //ia compute the damn chi2
    geodesic_optimizer.computeActiveErrors();
    if (kernel_type == "") {
      std::cerr << "reprojected chi2: " << geodesic_optimizer.chi2() << std::endl;
      std::cerr << "reprojected active chi2: " << geodesic_optimizer.activeChi2() << std::endl;
      //ia write to file
      comp_stats_stream << i << "; "
                        << chordal_optimizer.chi2() << "; "
                        << geodesic_optimizer.chi2();
    } else {
      //ia kernelized chi2
      std::cerr << "reprojected k-chi2: " << geodesic_optimizer.chi2() << std::endl;
      std::cerr << "reprojected k-active chi2: " << geodesic_optimizer.activeRobustChi2() << std::endl;
      //ia write to file
      comp_stats_stream << i << "; "
                        << chordal_optimizer.activeRobustChi2() << "; "
                        << geodesic_optimizer.activeRobustChi2();
    }
      
  }

  //ia close comparative stats
  comp_stats_stream.close();

  //ia write stats
  if (stats_filename != ""){
    std::cerr << "writing stats to file \"" << stats_filename << std::endl;
    ofstream os(stats_filename.c_str());
    const BatchStatisticsContainer& bsc = chordal_optimizer.batchStatistics();
    
    for (int i=0; i<max_iterations; i++) {
      os << bsc[i] << endl;
    }
  }

  //ia save the output
  cerr << "saving chordal output file : " << output_filename << endl;
  chordal_optimizer.save(output_filename.c_str());
  cerr << "done!" << endl;

  return 0;
}
