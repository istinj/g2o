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

      std::random_device rd;
      std::mt19937 generator(rd()); //ia not required until you need a lot of numbers

      //ia starting with points...
      _generateScatteredPoints(generator);
      //ia ...now lines...
      _generateScatteredLines(generator);
      //ia ..finally planes
      _generateScatteredPlanes(generator);
      
      _is_created = true;
    }

    void MatchableWorld::_generateScatteredPoints(std::mt19937& random_generator_) {
      std::uniform_real_distribution<> pos_distribution(0, 1);
      for (size_t i = 0; i < _num_points; ++i) {
        number_t x = pos_distribution(random_generator_)*_width;
        number_t y = pos_distribution(random_generator_)*_height;
        number_t z = pos_distribution(random_generator_);
        
        Matchable* point_matchable = new Matchable(Matchable::Point, Vector3(x,y,z));
        _landmarks.insert(point_matchable);
      }
    }

    void MatchableWorld::_generateScatteredLines(std::mt19937& random_generator_) {
      std::uniform_real_distribution<> pos_distribution(0, 1);

      //ia TODO: probabilistic axis selection
      //ia along x
      for (size_t i = 0; i < _num_lines/3; ++i) {
        number_t x = pos_distribution(random_generator_)*_width;
        number_t y = pos_distribution(random_generator_)*_height;
        number_t z = pos_distribution(random_generator_);
        
        Matchable* line_matchable = new Matchable(Matchable::Line, Vector3(x,y,z));
        line_matchable->computeRotationMatrixZXY(Vector3::UnitX());
        _landmarks.insert(line_matchable);
      }

      //ia along y
      for (size_t i = 0; i < _num_lines/3; ++i) {
        number_t x = pos_distribution(random_generator_)*_width;
        number_t y = pos_distribution(random_generator_)*_height;
        number_t z = pos_distribution(random_generator_);
        
        Matchable* line_matchable = new Matchable(Matchable::Line, Vector3(x,y,z));
        line_matchable->computeRotationMatrixZXY(Vector3::UnitY());
        _landmarks.insert(line_matchable);
      }

      //ia along z
      for (size_t i = 0; i < _num_lines/3; ++i) {
        number_t x = pos_distribution(random_generator_)*_width;
        number_t y = pos_distribution(random_generator_)*_height;
        number_t z = pos_distribution(random_generator_);
        
        Matchable* line_matchable = new Matchable(Matchable::Line, Vector3(x,y,z));
        line_matchable->computeRotationMatrixZXY(Vector3::UnitZ());
        _landmarks.insert(line_matchable);
      }

    }

    void MatchableWorld::_generateScatteredPlanes(std::mt19937& random_generator_) {
      std::uniform_real_distribution<> pos_distribution(0, 1);
      std::uniform_real_distribution<> axis_distribution(0,2.5);
      size_t i = 0;
      while (i < _num_planes) {
        number_t x = 0;
        number_t y = 0;
        number_t z = 0;

        //ia grid coords
        int r = 0, c = 0;
      
        Vector3 n = Vector3::Zero();
        Vector2I prev_cell = Vector2I::Zero();
        Vector2I curr_cell = Vector2I::Zero();

        Matchable* plane_matchable = 0; 
        
        const int axis_selector = round(axis_distribution(random_generator_));
        switch (axis_selector) {
        case (0):
          {
            n = Vector3::UnitX();
            r = round(pos_distribution(random_generator_)*_width);
            c = round(pos_distribution(random_generator_)*_height);

            x = r * _resolution;
            y = c * _resolution + _resolution/2.0;
          
            z = pos_distribution(random_generator_);

            plane_matchable = new Matchable(Matchable::Plane, Vector3(x,y,z));
            plane_matchable->computeRotationMatrixZXY(Vector3::UnitX());

            prev_cell.x() = r - 1;
            prev_cell.y() = c;
            curr_cell.x() = r;
            curr_cell.y() = c;
            break;
          }
        case (1):
          {
            n = Vector3::UnitY();
            r = round(pos_distribution(random_generator_)*_width);
            c = round(pos_distribution(random_generator_)*_height);

            x = r * _resolution + _resolution/2.0;
            y = c * _resolution;
          
            z = pos_distribution(random_generator_);

            plane_matchable = new Matchable(Matchable::Plane, Vector3(x,y,z));
            plane_matchable->computeRotationMatrixZXY(Vector3::UnitY());

            prev_cell.x() = r;
            prev_cell.y() = c - 1;
            curr_cell.x() = r;
            curr_cell.y() = c;
            break;
          }
        case (2):
          {
            n = Vector3::UnitZ();
            x = round(pos_distribution(random_generator_)*_width) + _resolution/2.0f;
            y = round(pos_distribution(random_generator_)*_height) + _resolution/2.0f;

            plane_matchable = new Matchable(Matchable::Plane, Vector3(x,y,z));
            plane_matchable->computeRotationMatrixZXY(Vector3::UnitZ());
            break;
          }
        default:
          break;
        }

        if (x - _resolution/2 < 0 || x + _resolution/2 >= _width ||
            y - _resolution/2 < 0 || y + _resolution/2 >= _height ||
            (z - _resolution/2 < 0 && axis_selector!= 2)) {
          continue;
        }

        if (!plane_matchable)
          throw std::runtime_error("no plane matchble");
        
        _landmarks.insert(plane_matchable);

        //ia wall
        if (axis_selector!=2) {
          CellPair pair0(prev_cell,curr_cell);
          CellPair pair1(curr_cell,prev_cell);
          _walls.insert(std::make_pair(pair0, plane_matchable));
          _walls.insert(std::make_pair(pair1, plane_matchable));
        }
        ++i;
      }

    }

    //ia maybe this is not useful anymore (can be done while moving)
    void MatchableWorld::removeWalls(int num_hits) {
      if (!_is_created)
        throw std::runtime_error("world has not been created. maybe you forgot to call createGrid()?");

      std::cerr << "creating a traversable path" << std::endl;
      int count=0;
      int sbrachi=0;
      bool continue_=true;

      //ia this fucking thing breaks valgrind so we will never know if there are leaks
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
        const Vector2I p = cell_pos.head(2).cast<int>();
        const Vector2I np = new_cell_pos.head(2).cast<int>();

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
