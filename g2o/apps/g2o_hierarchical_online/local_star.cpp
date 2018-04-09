#include "local_star.h"

//ia remove
#include "g2o/types/slam3d/types_slam3d.h"

namespace g2o {

LocalStar::LocalStar() {
  _gauge = 0;
  _max_size = 5;
  _level = 0;
  _gauge_dimension = -1;
}

LocalStar::~LocalStar() {
  // TODO Auto-generated destructor stub
}

StarHandler::StarHandler() {
  _stars_number = 0;
  _distance_thresh = 1.0;
}

StarHandler::~StarHandler() {
  for (LocalStar* s : _stars) {
    delete s;
  }
}

void StarHandler::init() {
  LocalStar* s = new LocalStar();
  _stars.push_back(s);
  ++_stars_number;
}

LocalStar* StarHandler::addStar() {
  LocalStar* star = new LocalStar();
  _stars.push_back(star);
  ++_stars_number;
  return _stars[_stars_number-1];
}

void StarHandler::addVertexToStar(HyperGraph::Vertex* v) {
  LocalStar* current_star = _stars[_stars_number-1];

  if (current_star->isFull()) {
    current_star = addStar();
  } else {
    //ia if the vertex is too far from the gauge, create another star
    if (current_star->gauge()) {
      const Isometry3& current_T = static_cast<VertexSE3*>(v)->estimate();
      const Isometry3& gauge_T = static_cast<VertexSE3*>(current_star->gauge())->estimate();
      Isometry3 diff_T = gauge_T.inverse() * current_T;

      std::cerr << "current_T.translation() " << current_T.translation().transpose() << std::endl;
      std::cerr << "gauge_T.translation() " << gauge_T.translation().transpose() << std::endl;
      std::cerr << "diff_T.translation() " << diff_T.translation().transpose() << std::endl;
      std::cerr << "diff_T.translation().norm() " << diff_T.translation().norm() << std::endl;

      if (_distance_thresh < diff_T.translation().norm()) {
        current_star = addStar();
      }
    }
  }

  current_star->addVertex(v);

  if(!current_star->gauge()) {
    current_star->setGauge(v->id());
  }

}

} /* namespace g2o */
