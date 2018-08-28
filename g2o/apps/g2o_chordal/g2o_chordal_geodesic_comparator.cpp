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

//ia include unscented
#include "g2o/stuff/unscented.h"

using namespace std;
using namespace g2o;

// @brief omega remapper
//ia useful typedefs
typedef std::pair<int, int> IntPair;
typedef SigmaPoint<Vector6> SigmaPoint6;
typedef SigmaPoint<Vector12> SigmaPoint12;

typedef std::vector<SigmaPoint<Vector6>,  Eigen::aligned_allocator<SigmaPoint<Vector6> > > SigmaPoint6Vector;
typedef std::vector<SigmaPoint<Vector12>, Eigen::aligned_allocator<SigmaPoint<Vector12> > > SigmaPoint12Vector;
Matrix12 _remapInformationMatrix(const Vector6& src_mean_,
                                 const Matrix6& src_omega_,
                                 const double& thresh_);

Matrix12 _reconditionateSigma(const Matrix12& src_sigma_,
                              const double& threshold_);

//! @brief this is called in the *preiteration* so
//!        this lives one iteration in the past :)
//!        If you make the stats from this guy, than
//!        you have to give an extra iteration (101 iterations)
struct ComparatorAction : public HyperGraphAction {

  //! @brief ptr to the geodesic opt
  SparseOptimizer* geo_opt_ptr = 0;
  //! @brief ptr to the chordal opt - also for stats
  SparseOptimizer* chord_opt_ptr = 0;
  //! @brief ptr to the stream where I will write the stats
  std::ofstream* stats = 0;
  //! @brief robust kernel flag
  bool kernel_flag = false;
  //! @brief dynamic epsilon flag for omegas
  bool dynamic_epsilon_flag = false;
  
  
  //! @breif ctor
  ComparatorAction(SparseOptimizer* geodesic_opt_,
                   SparseOptimizer* chordal_opt_,
                   std::ofstream* stats_stream_,
                   const bool kernel_flag_,
                   const bool dynamic_espilon_flag_) :
    geo_opt_ptr(geodesic_opt_),
    chord_opt_ptr(chordal_opt_),
    stats(stats_stream_),
    kernel_flag(kernel_flag_),
    dynamic_epsilon_flag(dynamic_espilon_flag_) {
    if (!geo_opt_ptr || !chord_opt_ptr)
      throw std::runtime_error("[ComparatorAction] you set a null pointer genious");
  }

  //! @brief all the shit is here :)
  HyperGraphAction* operator()(const HyperGraph* graph, Parameters* parameters) override {
    std::cerr << std::endl;
    int current_iteration = -10;
    //ia if everything is still null than do nothing
    if (!graph)
      return 0;

    //ia if there is an iteration number than print it in the stats
    ParametersIteration* param_it = dynamic_cast<ParametersIteration*>(parameters);
    if (param_it) {
      if (param_it->iteration < 0)
        return 0;

      current_iteration = param_it->iteration-1;
      std::cerr << "[ComparatorAction] iteration= " << current_iteration << std::endl;
    }

    //ia copy the damn vertices in the geodesic graph and compute the chi2
    HyperGraph::VertexIDMap::iterator v_it = chord_opt_ptr->vertices().begin();
    HyperGraph::VertexIDMap::iterator v_end = chord_opt_ptr->vertices().end();
    while (v_it != v_end) {
      //ia take the cordal vertex
      VertexSE3Chord* chordal_v = dynamic_cast<VertexSE3Chord*>(v_it->second);
      //ia take the corresponding geodesic vertex
      VertexSE3* geodesic_v = dynamic_cast<VertexSE3*>(geo_opt_ptr->vertex(v_it->first));

      if (!chordal_v || !geodesic_v)
        throw std::runtime_error("[ComparatorAction] dynamic cast failed, exit");

      //ia copy the damn estimate
      geodesic_v->setEstimate(chordal_v->estimate());
      
      ++v_it;
    }

    //ia compute the damn chi2
    geo_opt_ptr->computeActiveErrors();
    chord_opt_ptr->computeActiveErrors();
    
    if (!kernel_flag) {
      std::cerr << "[ComparatorAction] chordal active chi2= "
                << CL_YELLOW(FIXED(chord_opt_ptr->activeChi2())) << std::endl;
      std::cerr << "[ComparatorAction] reprojected active chi2: "
                << CL_GREEN(FIXED(geo_opt_ptr->activeChi2())) << std::endl;
      
      if (chord_opt_ptr->activeChi2() == 0) {
        return this;
      }

      //ia write to file
      (*stats) << "it= "<< current_iteration << "; "
               << "chordalChi2= " << FIXED(chord_opt_ptr->activeChi2()) << "; "
               << "reprojectedChi2= " << FIXED(geo_opt_ptr->activeChi2());
    } else {
      //ia kernelized chi2
      std::cerr << "[ComparatorAction] chordal k-active chi2= "
                << CL_YELLOW(FIXED(chord_opt_ptr->activeRobustChi2())) << std::endl;
      std::cerr << "[ComparatorAction] reprojected k-active chi2: "
                << CL_GREEN(FIXED(geo_opt_ptr->activeRobustChi2())) << std::endl;

      if (chord_opt_ptr->activeRobustChi2() == 0) {
        return this;
      }
      //ia write to file
      (*stats) << "it= "<< current_iteration << "; "
               << "chordalChi2= " << FIXED(chord_opt_ptr->activeRobustChi2()) << "; "
               << "reprojectedChi2= " << FIXED(geo_opt_ptr->activeRobustChi2());
    }

    (*stats) << std::endl;

    if (dynamic_epsilon_flag)  {
      //ia now we can adjust the omega for the next iteration
      const HyperGraph::EdgeSet& geo_edges = geo_opt_ptr->edges();
      HyperGraph::EdgeSet& chord_edges = chord_opt_ptr->edges();


      //ia standard epsilon
      number_t epsilon = 0.1;

      //ia adjust epsilon on the basis of the iterations
      if (current_iteration < 9) {
        epsilon = epsilon / (1*current_iteration+2);
      } else if (current_iteration >= 9 && current_iteration < 19) {
        epsilon = epsilon / (10*current_iteration+2);
      } else {
        epsilon = epsilon / (1000*current_iteration+2);
      }

      std::cerr << "dynamic epsilon = " << CL_LIGHTBLUE(epsilon) << std::endl;

      //ia remap information matrix
      HyperGraph::EdgeSet::const_iterator geo_edge_it = geo_edges.begin();
      HyperGraph::EdgeSet::iterator chord_edge_it = chord_edges.begin();
      
      while (geo_edge_it != geo_edges.end()) {
        EdgeSE3* geo_e = dynamic_cast<EdgeSE3*>(*geo_edge_it);
        EdgeSE3Chord* chord_e = dynamic_cast<EdgeSE3Chord*>(*chord_edge_it);

        std::pair<int, int> geo_ass(geo_e->vertices()[0]->id(), geo_e->vertices()[1]->id());
        std::pair<int, int> chord_ass(chord_e->vertices()[0]->id(), chord_e->vertices()[1]->id());

        if (geo_ass != chord_ass)
          throw std::runtime_error("edge mismatch e mo so cazzi");

        const Isometry3& geo_z = geo_e->measurement();
        const Matrix6& geo_omega = geo_e->information();

        Vector6 geo_z_minimal = internal::toVectorMQT(geo_z);
        Matrix12 chord_omega =  _remapInformationMatrix(geo_z_minimal, geo_omega, epsilon);

        chord_e->setInformation(chord_omega);

        ++geo_edge_it;
        ++chord_edge_it;
      }

    }

    return this;
  }
};


int main(int argc, char *argv[]) {
  // registering all the types from the libraries
  DlWrapper dlTypesWrapper;
  loadStandardTypes(dlTypesWrapper, argc, argv);
  // register all the solvers
  DlWrapper dlSolverWrapper;
  loadStandardSolver(dlSolverWrapper, argc, argv);

  // Command line parsing
  bool dynamic_epsilon;
  
  bool initial_guess;
  bool initial_guess_odometry;

  string initial_guess_type("no-guess");

  bool list_types_flag;
  bool list_solvers_flag;
  bool list_kernels_flag;
  
  int max_iterations;
  double kernel_width;
  
  string output_filename;
  string chordal_input_filename;
  string kernel_type;
  string comparison_stats_filename;
  string time_stats_filename;
  string summary_filaname;
  string geodesic_graph_filename;
  string solver_type;

  bool optimization_has_kernel = false;

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
  arg.param("summary", summary_filaname, "", "summary of the optimization - for tables :)");
  arg.param("timeStats", time_stats_filename, "", "stats for timing :)");
  arg.param("geodesicGraph", geodesic_graph_filename, "", "standard graph to compare - geodesic error");
  arg.param("dynamicEpsilon", dynamic_epsilon, false, "true to remap the omega dynamically");
  arg.paramLeftOver("graph-input", chordal_input_filename, "", "chordal graph file which will be processed");
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
    optimization_has_kernel = true;
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
  chordal_optimizer.setComputeBatchStatistics(true);

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
  geodesic_optimizer.setAlgorithm(solver_factory->construct(solver_type, solver_property_geodesic));
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
  geodesic_optimizer.setComputeBatchStatistics(true);



  // ---------------------------------------------------------------------------------- //
  // ---------------------------------------------------------------------------------- //
  // --------------------------- start the damn computation --------------------------- //
  // ---------------------------------------------------------------------------------- //
  // ---------------------------------------------------------------------------------- //  
  std::cerr << "comparison statistic file: " << comparison_stats_filename << std::endl;
  std::cerr << "stats legend: <iteration>; <chordal chi2>; <reprojected geodesic chi2>" << std::endl;
  std::ofstream comp_stats_stream(comparison_stats_filename);

  //ia install our custom action
  ComparatorAction* action = new ComparatorAction(&geodesic_optimizer,
                                                  &chordal_optimizer,
                                                  &comp_stats_stream,
                                                  optimization_has_kernel,
                                                  dynamic_epsilon);
  chordal_optimizer.addPreIterationAction(action);
  

  //ia initialize optimization
  chordal_optimizer.initializeOptimization();
  chordal_optimizer.computeActiveErrors();
  const double chordal_load_chi2 = chordal_optimizer.chi2(); 

  geodesic_optimizer.initializeOptimization();
  geodesic_optimizer.computeActiveErrors();
  const double geodesic_load_chi2 = geodesic_optimizer.chi2();
  
  std::cerr << "chordal graph load chi2  : " << FIXED(chordal_load_chi2) << std::endl;
  std::cerr << "geodesic graph load chi2 : " << FIXED(geodesic_load_chi2) << std::endl;

  //ia damn intial guess
  if (initial_guess && initial_guess_odometry)
    throw std::runtime_error("both initialization techniques selected, exit");
  
  if (initial_guess) {
    std::cerr << "computing initial guess from spanning tree" << std::endl;
    initial_guess_type = "spanning";
    chordal_optimizer.computeInitialGuess();
    // geodesic_optimizer.computeInitialGuess();
  }

  if (initial_guess_odometry) {
    std::cerr << "computing initial guess from odometry" << std::endl;
    initial_guess_type = "odometry";
    EstimatePropagatorCostOdometry chordal_odometry_cost_function(&chordal_optimizer);
    EstimatePropagatorCostOdometry geodesic_odometry_cost_function(&geodesic_optimizer);
    chordal_optimizer.computeInitialGuess(chordal_odometry_cost_function);
    // geodesic_optimizer.computeInitialGuess(geodesic_odometry_cost_function);
  }

  //ia get the damn initial chi2
  double initial_chi2 = -1.0;
  chordal_optimizer.computeActiveErrors();
  if (optimization_has_kernel)
    initial_chi2 = chordal_optimizer.activeRobustChi2();
  else
    initial_chi2 = chordal_optimizer.activeChi2();
  std::cerr << "initial chi2: " << FIXED(initial_chi2) << std::endl;


  //ua optimize
  int optimization_result = chordal_optimizer.optimize(max_iterations);
  if (optimization_result == OptimizationAlgorithm::Fail) {
    std::cerr << "cholesky failed" << std::endl;
  }
  

  //ia get the damn final chi2
  double final_chi2 = -1.0;
  chordal_optimizer.computeActiveErrors();
  if (optimization_has_kernel)
    final_chi2 = chordal_optimizer.activeRobustChi2();
  else
    final_chi2 = chordal_optimizer.activeChi2();


  //ia close comparative stats
  comp_stats_stream.close();

  //ia save the output
  if (output_filename != "") {
    cerr << "saving chordal output file : " << output_filename << endl;
    chordal_optimizer.save(output_filename.c_str());
    cerr << "done!" << endl;
  }

  //ia shit out a summary useful for latex tables
  if (summary_filaname != "") {
    std::cerr << "writing summary to file: " << summary_filaname << " ... ";
    PropertyMap summary;
    summary.makeProperty<StringProperty>("filename", chordal_input_filename);
    summary.makeProperty<IntProperty>("n_vertices", chordal_optimizer.vertices().size());
    summary.makeProperty<IntProperty>("n_edges", chordal_optimizer.edges().size());
    summary.makeProperty<StringProperty>("initial_guess_type", initial_guess_type);
    summary.makeProperty<IntProperty>("num_iterations", max_iterations-1);
    summary.makeProperty<StringProperty>("solver_type", solver_type);
    summary.makeProperty<DoubleProperty>("initial_chi", initial_chi2);
    summary.makeProperty<DoubleProperty>("final_chi", final_chi2);

    std::ofstream summary_stream(summary_filaname.c_str());
    summary.writeToCSV(summary_stream);
    summary_stream.close();
    std::cerr << "done." << std::endl;
  }

  //ia shit out the statitics
  if (time_stats_filename != "") {
    std::cerr << "writing time stats to file: " << time_stats_filename << " ... ";
    ofstream time_stats_stream(time_stats_filename.c_str());

    BatchStatisticsContainer complete_bsc = chordal_optimizer.batchStatistics();

    for (size_t i = 0; i < complete_bsc.size(); i++) {
      const G2OBatchStatistics& s = complete_bsc[i];
      time_stats_stream << "iteration= " << s.iteration << "; ";
      time_stats_stream << "preIterationTime= " << s.timePreIteration << "; ";
      time_stats_stream << "iterationTime= " << s.timeIteration << "; ";
      time_stats_stream << "completeIterationTime= " << s.timeCompleteIteration;
      time_stats_stream << std::endl;
    }
    std::cerr << "done." << std::endl;
    //ia porco dio close the stats
    time_stats_stream.close();
  }

  //ia clean-up
  delete action;
  return 0;
}






Matrix12 _remapInformationMatrix(const Vector6& src_mean_,
                                 const Matrix6& src_omega_,
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
  Matrix12 conditioned_sigma = _reconditionateSigma(remapped_sigma, threshold_);
  Matrix12 remapped_omega = conditioned_sigma.inverse();
  return remapped_omega;
}


Matrix12 _reconditionateSigma(const Matrix12& src_sigma_,
                              const double& threshold_) {

  Matrix12 conditioned_sigma = Matrix12::Zero();
  Eigen::JacobiSVD<Matrix12> svd(src_sigma_, Eigen::ComputeThinU | Eigen::ComputeThinV);
  double conditioned_eigenvalue = 1.0;
  for (int i = 0; i < 12; ++i) {
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
  return conditioned_sigma;
}
