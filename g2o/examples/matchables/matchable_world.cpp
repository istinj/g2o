#include "matchable_world.h"
#include "g2o/stuff/sampler.h"

namespace g2o {
  namespace matchables {

    MatchableWorld::MatchableWorld() {
      _params = Parameters();
      _removed_walls = 0;
      _is_valid = false;
    }
    MatchableWorld::~MatchableWorld() {
      for (Matchable* m : _landmarks) {
        delete m;
      }
    }

    //ia I think we can reduce the number of lines and planes
    void MatchableWorld::createGrid() {
      if (_params.num_points == 0 &&
          _params.num_lines == 0 &&
          _params.num_planes == 0) {
        std::cerr << "no landmarks selected" << std::endl;
        return;
      }

      if (_landmarks.size())
        throw std::runtime_error("world alredy initialized");

      _removed_walls = 0;

      std::cerr << "\ncreate a grid world with the selected parameters" << std::endl;

      std::random_device rd;
      std::mt19937 generator(rd()); //ia not required until you need a lot of numbers

      //ia starting with points...
      _generateScatteredPoints(generator);
      //ia ...now lines...
      _generateScatteredLines(generator);
      //ia ..finally planes
      _generateScatteredPlanes(generator);

      std::cerr << "done" << std::endl << std::endl;
      
      _is_valid = true;
    }

    void MatchableWorld::_generateScatteredPoints(std::mt19937& random_generator_) {
      uint32_t cnt = 0;
      std::uniform_real_distribution<> uniform_distribution(0, 1);
      for (size_t i = 0; i < _params.num_points; ++i) {
        number_t x = uniform_distribution(random_generator_)*_params.width;
        number_t y = uniform_distribution(random_generator_)*_params.height;
        number_t z = uniform_distribution(random_generator_);
        
        Matchable* point_matchable = new Matchable(Matchable::Point, Vector3(x,y,z));
        _landmarks.insert(point_matchable);
        ++cnt;
      }

      std::cerr << "generated " << cnt << " points" << std::endl;
    }

    void MatchableWorld::_generateScatteredLines(std::mt19937& random_generator_) {
      std::uniform_real_distribution<> uniform_distribution(0, 1);

      // uint32_t xcnt = 0;
      // uint32_t ycnt = 0;
      // uint32_t zcnt = 0;
      uint32_t cnt = 0;
      
      for (size_t i = 0; i < _params.num_lines; ++i) {
        number_t x = uniform_distribution(random_generator_)*_params.width;
        number_t y = uniform_distribution(random_generator_)*_params.height;
        number_t z = uniform_distribution(random_generator_);
        
        Vector3 axis = Vector3::Zero();
        const double axis_selector = uniform_distribution(random_generator_);
        if (axis_selector < _params.line_axis_probabilities.x_axis_prob) {
          axis = Vector3::UnitX();
          // ++ xcnt;
        } else if ((axis_selector <
                   _params.line_axis_probabilities.x_axis_prob+
                   _params.line_axis_probabilities.y_axis_prob) &&
                   _params.line_axis_probabilities.x_axis_prob < axis_selector) {
          axis = Vector3::UnitY();
          // ++ ycnt;
        } else {
          axis = Vector3::UnitZ();
          // ++ zcnt;
        }
        
        Matchable* line_matchable = new Matchable(Matchable::Line, Vector3(x,y,z));
        line_matchable->computeRotationMatrixZXY(axis);
        _landmarks.insert(line_matchable);
        ++cnt;
      }

      // std::cerr << "lines ->\tx: " << xcnt << "\ty: " << ycnt << "\tz: " << zcnt << std::endl;
      std::cerr << "generated " << cnt << " lines" << std::endl;
    }

    void MatchableWorld::_generateScatteredPlanes(std::mt19937& random_generator_) {
      std::uniform_real_distribution<> uniform_distribution(0, 1);
      
      size_t i = 0;
      // uint32_t xcnt = 0;
      // uint32_t ycnt = 0;
      // uint32_t zcnt = 0;
      
      while (i < _params.num_planes) {
        //ia world coords
        number_t x = 0;
        number_t y = 0;
        number_t z = 0;

        //ia grid coords
        int r = 0, c = 0;
        Vector2I prev_cell = Vector2I::Zero();
        Vector2I curr_cell = Vector2I::Zero();

        //ia normals
        Vector3 n = Vector3::Zero();
        Matchable* plane_matchable = 0;


        const double selector = uniform_distribution(random_generator_);
        if (selector < _params.plane_axis_probabilities.x_axis_prob) {
          n = Vector3::UnitX();
          r = round(uniform_distribution(random_generator_)*_params.width);
          c = round(uniform_distribution(random_generator_)*_params.height);

          x = r * _params.resolution;
          y = c * _params.resolution + _params.resolution/2.0;
          
          z = uniform_distribution(random_generator_);

          plane_matchable = new Matchable(Matchable::Plane, Vector3(x,y,z));
          plane_matchable->computeRotationMatrixZXY(Vector3::UnitX());

          prev_cell.x() = r - 1;
          prev_cell.y() = c;
          curr_cell.x() = r;
          curr_cell.y() = c;
          // ++xcnt;
        } else if ((selector <
                   _params.plane_axis_probabilities.x_axis_prob+
                   _params.plane_axis_probabilities.y_axis_prob) &&
                   _params.plane_axis_probabilities.x_axis_prob < selector) {
          //ia y axis
          n = Vector3::UnitY();
          r = round(uniform_distribution(random_generator_)*_params.width);
          c = round(uniform_distribution(random_generator_)*_params.height);

          x = r * _params.resolution + _params.resolution/2.0;
          y = c * _params.resolution;
          
          z = uniform_distribution(random_generator_);

          plane_matchable = new Matchable(Matchable::Plane, Vector3(x,y,z));
          plane_matchable->computeRotationMatrixZXY(Vector3::UnitY());

          prev_cell.x() = r;
          prev_cell.y() = c - 1;
          curr_cell.x() = r;
          curr_cell.y() = c;
          // ++ycnt;
        } else {
          // z axis (floor)
          n = Vector3::UnitZ();
          x = round(uniform_distribution(random_generator_)*_params.width) + _params.resolution/2.0f;
          y = round(uniform_distribution(random_generator_)*_params.height) + _params.resolution/2.0f;

          plane_matchable = new Matchable(Matchable::Plane, Vector3(x,y,z));
          plane_matchable->computeRotationMatrixZXY(Vector3::UnitZ());
          // ++zcnt;
        }

        if (x - _params.resolution/2 < 0 || x + _params.resolution/2 >= _params.width ||
            y - _params.resolution/2 < 0 || y + _params.resolution/2 >= _params.height ||
            (z - _params.resolution/2 < 0 && selector<_params.plane_axis_probabilities.z_axis_prob)) {
          continue;
        }
      
        if (!plane_matchable)
          throw std::runtime_error("no plane matchble");
        
        _landmarks.insert(plane_matchable);

        //ia wall
        if (selector < _params.plane_axis_probabilities.z_axis_prob) {
          CellPair pair0(prev_cell,curr_cell);
          CellPair pair1(curr_cell,prev_cell);
          _walls.insert(std::make_pair(pair0, plane_matchable));
          _walls.insert(std::make_pair(pair1, plane_matchable));
        }
        ++i;
      }

      // std::cerr << "planes ->\tx: " << xcnt << "\ty: " << ycnt << "\tz: " << zcnt << std::endl;
      std::cerr << "generated " << i << " planes" << std::endl;
    }

  } //ia end namespace matchables
} //ia end namespace g2o
