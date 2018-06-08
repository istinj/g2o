#include "matchable_world.h"
#include "g2o/stuff/sampler.h"

namespace g2o {
  namespace matchables {

    MatchableWorld::MatchableWorld() {
      _is_created = false;
    }
    MatchableWorld::~MatchableWorld() {
      for (Matchable* m : _landmarks) {
        delete m;
      }
    }

    //ia I think we can reduce the number of lines and planes
    void MatchableWorld::createGrid() {
      if (_num_points == 0 && _num_lines == 0 && _num_planes == 0) {
        std::cerr << "no landmarks selected" << std::endl;
        return;
      }

      //ia sampling landmarks
      std::random_device rd;
      std::mt19937 gen(rd()); //ia not required until you need a lot of numbers
      std::uniform_real_distribution<> dis(0, 1);
      int points=0,lines=0,planes=0;
      
      //ia starting with points
      for (size_t i = 0; i < _num_points; ++i) {
        number_t x = dis(gen)*_width;
        number_t y = dis(gen)*_height;
        number_t z = dis(gen);
        
        Matchable* point_matchable = new Matchable(Matchable::Point, Vector3(x,y,z));
        _landmarks.insert(point_matchable);

        ++points;
      }

      //ia along x
      for (size_t i = 0; i < _num_lines/3; ++i) {
        number_t x = dis(gen)*_width;
        number_t y = dis(gen)*_height;
        number_t z = dis(gen);
        
        Matchable* line_matchable = new Matchable(Matchable::Line, Vector3(x,y,z));
        line_matchable->computeRotationMatrixZXY(Vector3::UnitX());
        _landmarks.insert(line_matchable);
        lines++;
      }

      //ia along y
      for (size_t i = 0; i < _num_lines/3; ++i) {
        number_t x = dis(gen)*_width;
        number_t y = dis(gen)*_height;
        number_t z = dis(gen);
        
        Matchable* line_matchable = new Matchable(Matchable::Line, Vector3(x,y,z));
        line_matchable->computeRotationMatrixZXY(Vector3::UnitY());
        _landmarks.insert(line_matchable);
        lines++;
      }

      //ia along z
      for (size_t i = 0; i < _num_lines/3; ++i) {
        number_t x = dis(gen)*_width;
        number_t y = dis(gen)*_height;
        number_t z = dis(gen);
        
        Matchable* line_matchable = new Matchable(Matchable::Line, Vector3(x,y,z));
        line_matchable->computeRotationMatrixZXY(Vector3::UnitZ());
        _landmarks.insert(line_matchable);
        lines++;
      }

      //insert planes along x axes
      for(size_t j=0; j<_height-1; j+=3) {
        for(size_t i=0; i<_width; i+=5){
          const Eigen::Vector2i cell(i,j);
          const Vector3 p(cell.x()*_resolution,
                          cell.y()*_resolution+_resolution/2.0f,
                          _resolution/2.0f);
          
          Matchable* plane_matchable = new Matchable(Matchable::Plane,p);
          plane_matchable->computeRotationMatrixZXY(Vector3::UnitX());
          _landmarks.insert(plane_matchable);
          planes++;

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
      for(size_t i=0; i<_width-1; i+=3) {
        for(size_t j=0; j<_height; j+=5){
          const Eigen::Vector2i cell(i,j);
          const Vector3 p(cell.x()*_resolution+_resolution/2.0f,
                          cell.y()*_resolution,
                          _resolution/2.0f);
          Matchable* plane_matchable = new Matchable(Matchable::Plane,p);
          plane_matchable->computeRotationMatrixZXY(Vector3::UnitY());
          _landmarks.insert(plane_matchable);
          planes++;

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
      for(size_t j=0; j<_height-1; j+=3) {
        for(size_t i=0; i<_width-1; i+=3){
          const Vector3 p(i*_resolution+_resolution/2.0f,
                          j*_resolution+_resolution/2.0f,
                          0.0f);
          Matchable* plane_matchable = new Matchable(Matchable::Plane,p);
          plane_matchable->computeRotationMatrixZXY(Vector3::UnitZ());
          _landmarks.insert(plane_matchable);
          planes++;
        }
      }

      std::cerr << "Created grid with " << _landmarks.size() << " matchables" << std::endl;
      std::cerr << "Points: " << points << std::endl;
      std::cerr << "Lines : " << lines  << std::endl;
      std::cerr << "Planes: " << planes << std::endl;

      _is_created = true;
    }

    void MatchableWorld::removeWalls(int num_hits) {
      if (!_is_created)
        throw std::runtime_error("world has not been created. maybe you forgot to call createGrid()?");

      std::cerr << "creating a traversable path" << std::endl;
      int count=0;
      int sbrachi=0;
      bool continue_=true;

      // //ia this fucking thing breaks valgrind so we will never know if there are leaks
      std::random_device rd;
      std::mt19937 gen(rd()); //ia not required until you need a lot of numbers
      std::uniform_real_distribution<> dis(0, 1);

      number_t n = 0;
      Vector3 increment = Vector3::Zero();

      Vector3 new_position = Vector3::Zero();
      Vector3 new_cell_pos = Vector3::Zero();

      Vector3 position((_width/2-1)*_resolution,(_height/2-1)*_resolution,0.0f);
      Vector3 cell_pos(_width/2-1, _height/2-1, 0);

      //ia this rand is here to run valgrind :)
      // Vector3 rand;
      while(continue_){
        std::cerr << "x";
        //sample new position
        n = dis(gen);
        // rand.setRandom();
        // n = rand.x();

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

        new_cell_pos = cell_pos+increment;

        //check if new position is out of grid
        if(new_cell_pos.x() < 0.0f || new_cell_pos.x() > (number_t)(_width-1) ||
           new_cell_pos.y() < 0.0f || new_cell_pos.y() > (number_t)(_height-1)) {
          continue;
        }

        new_position.head(2) = new_cell_pos.head(2)*_resolution;
        new_position.z() = new_cell_pos.z();

        //sbraco
        const Eigen::Vector2i p = cell_pos.head(2).cast<int>();
        const Eigen::Vector2i np = new_cell_pos.head(2).cast<int>();

        CellPair pair(p,np);
        CellPairPlaneMap::iterator it = _walls.find(pair);

        if(it != _walls.end()) {
          Matchable* m = it->second;
          MatchableSet::iterator jt = _landmarks.find(m);
          if(jt != _landmarks.end()){
            _landmarks.erase(jt);
            sbrachi++;
          }
        }

        position = new_position;
        cell_pos = new_cell_pos;
        count++;

        if(count == num_hits)
          continue_=false;

      }
      std::cerr << std::endl << std::endl;
      // std::cerr << "removed " << sbrachi << " planes!" << std::endl;
      _is_valid = true;
    }
  }
}
