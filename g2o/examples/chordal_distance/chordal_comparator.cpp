#include <iostream>
#include <string>

//ia include types
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"

//ia include g2o core stuff
#include "g2o/stuff/command_args.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/core/batch_stats.h"
#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_factory.h"

using namespace std;
using namespace g2o;

int main(int argc, char *argv[]) {
  // required for type factory
  VertexSE3* v_quat = new VertexSE3();
  EdgeSE3* e_quat = new EdgeSE3();
  VertexSE3Chord* v_chord = new VertexSE3Chord();
  EdgeSE3Chord* e_chord = new EdgeSE3Chord();

  // Command line parsing
  int maxIterations;
  string outputFilename;
  string inputFilename;
  string robustKernel;
  string statsFile;
  string compareStatsFile;
  double huberWidth; 
  string otherGraph;

  g2o::CommandArgs arg;
  arg.param("i", maxIterations, 10, "perform n iterations, if negative consider the gain");
  arg.param("o", outputFilename, "", "output final version of the graph");
  arg.param("robustKernel", robustKernel, "", "use this robust error function");
  arg.param("robustKernelWidth", huberWidth, -1., "width for the robust Kernel (only if robustKernel)");
  arg.param("compareStats", compareStatsFile, "", "specify a file for comparison stats");
  arg.param("stats", statsFile, "", "specify a file for the statistics");
  arg.param("otherGraph", otherGraph, "", "graph to compare");
  arg.paramLeftOver("graph-input", inputFilename, "", "graph file which will be processed");
  arg.parseArgs(argc, argv);

  if (otherGraph == "") {
    cerr << "forgot otherGraph" << endl;
    return 0;
  }

  if (compareStatsFile == "") {
    cerr << "forgot compareStatsFile" << endl;
    return 0;
  }

  typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> >  SlamBlockSolver;
  typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
  // typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  g2o::SparseOptimizer optimizer;

  auto linear_solver = g2o::make_unique<SlamLinearSolver>();
  linear_solver->setBlockOrdering(true);

  g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(
    g2o::make_unique<SlamBlockSolver>(move(linear_solver)));

  //ia set optimizer things
  optimizer.setVerbose(true);
  optimizer.setAlgorithm(solver);
  if (statsFile!=""){
    // allocate buffer for statistics;
    optimizer.setComputeBatchStatistics(true);
  }

  //ia loading the files
  cerr << "opening file : " << inputFilename << endl;
  ifstream ifs(inputFilename.c_str());
  optimizer.load(ifs);
  int optimizer_num_v = optimizer.vertices().size();
  int optimizer_num_e = optimizer.edges().size();
  cerr << "Loaded " << optimizer_num_v << " vertices" << endl;
  cerr << "Loaded " << optimizer_num_e << " edges" << endl;

  //ia kernelize
  if (robustKernel.size() > 0) {
    AbstractRobustKernelCreator* creator = RobustKernelFactory::instance()->creator(robustKernel);
    cerr << "# Preparing robust error function ... ";
    if (creator) {
      for (SparseOptimizer::EdgeSet::iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
        SparseOptimizer::Edge* e = dynamic_cast<SparseOptimizer::Edge*>(*it);
        e->setRobustKernel(creator->construct());
        if (huberWidth > 0)
          e->robustKernel()->setDelta(huberWidth);
      }
      cerr << "done." << endl;
    } else {
      cerr << "Unknown Robust Kernel: " << robustKernel << endl;
    }
  }

  // ---------------------------------------------------------------------------------- //
  // ---------------------------------------------------------------------------------- //
  //ia other optimizer setup
  // ---------------------------------------------------------------------------------- //
  // ---------------------------------------------------------------------------------- //
  g2o::SparseOptimizer other_optimizer;
  auto other_linear_solver = g2o::make_unique<SlamLinearSolver>();
  other_linear_solver->setBlockOrdering(true);

  g2o::OptimizationAlgorithmGaussNewton* other_solver = new g2o::OptimizationAlgorithmGaussNewton(
    g2o::make_unique<SlamBlockSolver>(move(other_linear_solver)));
  other_optimizer.setVerbose(true);
  other_optimizer.setAlgorithm(other_solver);
  if (statsFile!=""){
    other_optimizer.setComputeBatchStatistics(true);
  }
  cerr << "opening file : " << otherGraph << endl;
  ifstream other_ifs(otherGraph.c_str());
  other_optimizer.load(other_ifs);
  int other_optimizer_num_v = other_optimizer.vertices().size();
  int other_optimizer_num_e = other_optimizer.edges().size();
  cerr << "Loaded " << other_optimizer_num_v << " vertices" << endl;
  cerr << "Loaded " << other_optimizer_num_e << " edges" << endl;

  if (other_optimizer_num_e != optimizer_num_e || 
    other_optimizer_num_v != optimizer_num_v) {
    cerr << "graph must be the same" << endl;
  }

  if (robustKernel.size() > 0) {
    AbstractRobustKernelCreator* creator = RobustKernelFactory::instance()->creator(robustKernel);
    cerr << "# Preparing robust error function ... ";
    if (creator) {
      for (SparseOptimizer::EdgeSet::iterator it = other_optimizer.edges().begin(); it != other_optimizer.edges().end(); ++it) {
        SparseOptimizer::Edge* e = dynamic_cast<SparseOptimizer::Edge*>(*it);
        e->setRobustKernel(creator->construct());
        if (huberWidth > 0)
          e->robustKernel()->setDelta(huberWidth);
      }
      cerr << "done." << endl;
    } else {
      cerr << "Unknown Robust Kernel: " << robustKernel << endl;
    }
  }

  //ia write chi2 squares in a file
  //file: <it>; <chordal_chi2>; <reprojected standard_chi2>
  ofstream comp_stats_file(compareStatsFile);

  //ia perform optimization
  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();

  other_optimizer.initializeOptimization();
  other_optimizer.computeActiveErrors();

  double chordal_loadChi = optimizer.chi2();
  double standard_loadChi = other_optimizer.chi2();
  cerr << "Chordal initial chi2  = " << FIXED(chordal_loadChi) << endl;
  cerr << "Standard initial chi2 = " << FIXED(standard_loadChi) << endl;
  // comp_stats_file << 0 << "; " << chordal_loadChi << "; " << standard_chi2 << endl; //ia maybe not useful

  for (int i = 0; i < maxIterations; ++i) {
    optimizer.optimize(1);
    for (std::pair<const int, g2o::HyperGraph::Vertex*> vit : optimizer.vertices()) {
      VertexSE3Chord* current_vertex = dynamic_cast<VertexSE3Chord*>(vit.second);
      VertexSE3* other_vertex = dynamic_cast<VertexSE3*>(other_optimizer.vertex(current_vertex->id()));
      other_vertex->setEstimate(current_vertex->estimate());
    }
    other_optimizer.computeActiveErrors();
    if (robustKernel.size() > 0)
      cerr << "Standard robust active chi2= " << other_optimizer.activeRobustChi2() << endl << endl;
    else
      cerr << "Standard active chi2= " << other_optimizer.activeChi2() << endl << endl;
    if (robustKernel.size() > 0)
      comp_stats_file << i << "; " << optimizer.activeRobustChi2() << "; " << other_optimizer.activeRobustChi2() << endl;
    else 
      comp_stats_file << i << "; " << optimizer.chi2() << "; " << other_optimizer.chi2() << endl;
  }


  //ia write stats
  if (statsFile!=""){
    cerr << "writing stats to file \"" << statsFile << "\" ... ";
    ofstream os(statsFile.c_str());
    const BatchStatisticsContainer& bsc = optimizer.batchStatistics();
    
    for (int i=0; i<maxIterations; i++) {
      os << bsc[i] << endl;
    }
    cerr << "done." << endl;
  }

  cerr << "saving output file : " << outputFilename << endl;
  optimizer.save(outputFilename.c_str());
  comp_stats_file.close();
  cerr << "done!" << endl;

  // checkout things
  delete v_quat;
  delete e_quat;
  delete v_chord;
  delete e_chord;
  return 0;
}
