#include "matchable_world.h"

namespace g2o {
  namespace matchables {

    //ia aid function
    Matrix3 _computeRotationMatrixZXY(const Vector3& direction_) {
      number_t d = sqrt(direction_.x()*direction_.x() + direction_.y()*direction_.y());

      const number_t& dirx = direction_.x();
      const number_t& diry = direction_.y();
      const number_t& dirz = direction_.z();

      Matrix3 rotation_matrix;
      if(d > std::numeric_limits<number_t>::min()) {
        rotation_matrix <<
          dirx, diry/d,  dirx*dirz/d,
          diry, -dirx/d, diry*dirz/d,
          dirz, 0,       -d;
      } else {
        rotation_matrix.setIdentity();
      }
      return rotation_matrix;
    }
    
    MatchableWorld::MatchableWorld() {}
    MatchableWorld::~MatchableWorld() {}

    void MatchableWorld::createGrid() {      

      //insert points
      for(size_t j=0; j<_height; ++j) {
        for(size_t i=0; i<_width; ++i) {
          const Vector3 p(i*_resolution, j*_resolution, 0.0f);
          MatchablePtr point_matchable(new Matchable(Matchable::Point,p));
          _landmarks.insert(point_matchable);
        }
      }

      //insert lines along x axes
      for(size_t j=0; j<_height; ++j) {
        for(size_t i=0; i<_width-1; ++i){
          const Vector3 p(i*_resolution, j*_resolution, 0.0f);
          const Matrix3 R = _computeRotationMatrixZXY(Vector3::UnitX());
          MatchablePtr line_matchable(new Matchable(Matchable::Line,p,R));
          _landmarks.insert(line_matchable);
        }
      }

      //insert lines along y axes
      for(size_t i=0; i<_width; ++i) {
        for(size_t j=0; j<_height-1; ++j){
          const Vector3 p(i*_resolution, j*_resolution, 0.0f);
          const Matrix3 R = _computeRotationMatrixZXY(Vector3::UnitY());
          MatchablePtr line_matchable(new Matchable(Matchable::Line,p,R));
          _landmarks.insert(line_matchable);
        }
      }

      //insert lines along z axes
      for(size_t j=0; j<_height; ++j) {
        for(size_t i=0; i<_width; ++i){
          const Vector3 p(i*_resolution, j*_resolution, 0.0f);
          const Matrix3 R = _computeRotationMatrixZXY(Vector3::UnitZ());
          MatchablePtr line_matchable(new Matchable(Matchable::Line,p,R));
          _landmarks.insert(line_matchable);
        }
      }

      //insert planes along x axes
      for(size_t j=0; j<_height-1; ++j) {
        for(size_t i=0; i<_width; ++i){
          const Eigen::Vector2i cell(i,j);
          const Vector3 p(cell.x()*_resolution, cell.y()*_resolution+_resolution/2.0f, _resolution/2.0f);
          const Matrix3 R = _computeRotationMatrixZXY(Vector3::UnitX());
          MatchablePtr plane_matchable(new Matchable(Matchable::Plane,p,R));
          _landmarks.insert(plane_matchable);

          if(i > 0 && i < _width-1){
            const Eigen::Vector2i left_cell(i-1,j);
            CellPair left_pair(left_cell,cell);
            _walls[left_pair] = plane_matchable;
            CellPair right_pair(cell,left_cell);
            _walls[right_pair] = plane_matchable;
          }
        }
      }

      //insert planes along y axes
      for(size_t i=0; i<_width-1; ++i) {
        for(size_t j=0; j<_height; ++j){
          const Eigen::Vector2i cell(i,j);
          const Vector3 p(cell.x()*_resolution+_resolution/2.0f, cell.y()*_resolution, _resolution/2.0f);
          const Matrix3 R = _computeRotationMatrixZXY(Vector3::UnitY());
          MatchablePtr plane_matchable(new Matchable(Matchable::Plane,p,R));
          _landmarks.insert(plane_matchable);

          if(j > 0 && j < _height-1){
            const Eigen::Vector2i down_cell(i,j-1);
            CellPair down_pair(down_cell,cell);
            _walls[down_pair] = plane_matchable;
            CellPair up_pair(cell,down_cell);
            _walls[up_pair] = plane_matchable;
          }
        }
      }

      //insert planes along z axes
      for(size_t j=0; j<_height-1; ++j) {
        for(size_t i=0; i<_width-1; ++i){
          const Vector3 p(i*_resolution+_resolution/2.0f, j*_resolution+_resolution/2.0f, 0.0f);
          const Matrix3 R = _computeRotationMatrixZXY(Vector3::UnitZ());
          MatchablePtr plane_matchable(new Matchable(Matchable::Plane,p,R));
          _landmarks.insert(plane_matchable);
        }
      }

      std::cerr << "World has " << _landmarks.size() << " matchables before sbraco" << std::endl;
    }

    void MatchableWorld::removeWalls(int num_hits){

      std::cerr << "sbraco..." << std::endl;
      int count=0;
      bool continue_=true;
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<> dis(0, 1);

      number_t n = 0;
      Vector3 increment = Vector3::Zero();
      Vector3 new_position = Vector3::Zero();
      Vector3 position(_width/2-1,_height/2-1,0.0f);
      Isometry3 pose = Isometry3::Identity();
      pose.translation() = Vector3(position.x(), position.y(), 0.1);
      Matrix3 R;
      R = AngleAxis(position.z(), Vector3::UnitZ());
      pose.linear() = R;

      while(continue_){
        //sample new position
        n = dis(gen);

        //go forward
        if(n < 0.5f){
          increment.x() = round(cos(position.z()));
          increment.y() = round(sin(position.z()));
        }
        //go left
        if(n >= 0.5f && n < 0.75f){
          increment.x() = round(-sin(position.z()));
          increment.y() = round(cos(position.z()));
          increment.z() = M_PI/2.0f;
        }
        //go right
        if(n >= 0.75f){
          increment.x() = round(sin(position.z()));
          increment.y() = round(-cos(position.z()));
          increment.z() = -M_PI/2.0f;
        }

        new_position = position+increment;

        //check if new position is out of grid
        if(new_position.x() < 0.0f || new_position.x() > (number_t)(_width-1) ||
           new_position.y() < 0.0f || new_position.y() > (number_t)(_height-1)) {
          continue;
        }

        //sbraco
        const Eigen::Vector2i p = position.head(2).cast<int>();
        const Eigen::Vector2i np = new_position.head(2).cast<int>();
        CellPair pair(p,np);
        CellPairPlaneMap::iterator it = _walls.find(pair);

        if(it != _walls.end()) {
          MatchablePtr m = it->second;
          MatchablePtrSet::iterator jt = _landmarks.find(m);
          if(jt != _landmarks.end())
            _landmarks.erase(jt);
        } else {
          std::cerr << pair.first_cell.transpose() << " - " << pair.second_cell.transpose() << " not found!" << std::endl;
        }

        position = new_position;
        count++;

        if(count == num_hits)
          continue_=false;

      }
    }
  }
}
