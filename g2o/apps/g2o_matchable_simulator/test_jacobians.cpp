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
// #include "g2o/core/block_solver.h"
// #include "g2o/core/optimization_algorithm_factory.h"
// #include "g2o/core/optimization_algorithm_levenberg.h"
// #include "g2o/solvers/cholmod/linear_solver_cholmod.h"
// #include "g2o/solvers/csparse/linear_solver_csparse.h"
// #include "g2o/core/batch_stats.h"
#include "g2o/core/factory.h"

#include <Eigen/Core>
#include <Eigen/StdVector>

using namespace std;
using namespace g2o;
using namespace matchables;
// typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
// typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

typedef Eigen::Matrix<number_t, 7, 6> JacobianIType;
typedef Eigen::Matrix<number_t, 7, 5> JacobianJType;

int main (int argc, char** argv) {
  EdgeSE3Chord e_se3;
  EdgeSE3Matchable e_match;
  std::string e_matchable_tag = Factory::instance()->tag(&e_match);
  std::string e_se3_tag = Factory::instance()->tag(&e_se3);
  
  std::string input_filename;
  
  CommandArgs arg;
  arg.paramLeftOver("graph-input", input_filename, "", "input g2o file", true);
  arg.parseArgs(argc, argv);

  //ia load the graph
  SparseOptimizer optimizer;
  // auto linear_solver = g2o::make_unique<SlamLinearSolver>();
  // linear_solver->setBlockOrdering(true);

  // g2o::OptimizationAlgorithmLevenberg* solver =
  //     new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<SlamBlockSolver>(std::move(linear_solver)));
  // optimizer.setAlgorithm(solver);

  
  std::cerr << "opening file : " << input_filename << std::endl;
  std::ifstream ifs(input_filename.c_str());
  optimizer.load(ifs);

  // optimizer.initializeOptimization();
  // optimizer.computeActiveErrors();

  HyperGraph::EdgeSet& edges = optimizer.edges();

  for (HyperGraph::Edge* e : edges) {
    if (Factory::instance()->tag(e) == e_se3_tag) {
      std::cerr << "skip se3 edges" << std::endl;
      continue;
    }

    if (Factory::instance()->tag(e) == e_matchable_tag) {
      std::cerr << "edge matchable" << std::endl;
      EdgeSE3Matchable* edge = dynamic_cast<EdgeSE3Matchable*>(e);

      JacobianIType j_i_numerical = JacobianIType::Zero();
      JacobianJType j_j_numerical = JacobianJType::Zero();

      JacobianIType j_i_analytical = JacobianIType::Zero();
      JacobianJType j_j_analytical = JacobianJType::Zero();

      //ia analytical
      // edge->linearizeOplus();
      edge->analyticalJacobians(j_i_analytical, j_j_analytical);

      //ia numerical
      edge->numericalJacobians(j_i_numerical, j_j_numerical);

      std::cerr << "**************** analical jacobians *****************" << std::endl;
      std::cerr << "Ji:\n" << j_i_analytical << std::endl;
      std::cerr << "Jj:\n" << j_j_analytical << std::endl;

      std::cerr << "**************** numerical jacobians *****************" << std::endl;
      std::cerr << "Ji:\n" << j_i_numerical << std::endl;
      std::cerr << "Jj:\n" << j_j_numerical << std::endl;

      std::cerr << "**************** difference *****************" << std::endl;
      std::cerr << "Ji:\n" << j_i_analytical - j_i_numerical << std::endl;
      std::cerr << "Jj:\n" << j_j_analytical - j_j_numerical << std::endl;
      
      
    }

    std::cerr << std::endl;
    std::cin.get();
  }
  
  
  return 0;
}
