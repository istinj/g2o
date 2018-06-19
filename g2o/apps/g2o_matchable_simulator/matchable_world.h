#pragma once
#include <set>
#include <map>
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/matchables3d/types_matchables.h"

namespace g2o {
  namespace matchables {

    //!TODO:
    //3 remove "removeWalls" function and move this phase in the motion simulation phase

    struct CellPair {
      CellPair(const Eigen::Vector2i& first_cell_,
               const Eigen::Vector2i& second_cell_):
        first_cell(first_cell_),
        second_cell(second_cell_){}

      inline bool operator <(const CellPair &c) const {
        for(int i=0; i<2; ++i){
          if(first_cell[i] < c.first_cell[i])
            return true;
          if(first_cell[i] > c.first_cell[i])
            return false;
        }
        for(int i=0; i<2; ++i){
          if(second_cell[i] < c.second_cell[i])
            return true;
          if(second_cell[i] > c.second_cell[i])
            return false;
        }
        return false;
      }

      inline bool operator ==(const CellPair& c) const {
        for(int i=0; i<2; ++i)
          if(first_cell[i] != c.first_cell[i])
            return false;

        for(int i=0; i<2; ++i)
          if(second_cell[i] != c.second_cell[i])
            return false;

        return true;
      }

      Eigen::Vector2i first_cell;
      Eigen::Vector2i second_cell;
    };
    
    typedef std::map<CellPair, Matchable*> CellPairPlaneMap;
    
    struct AxisProbability {
      AxisProbability() {
        x_axis_prob = 0.33;
        y_axis_prob = 0.33;
        z_axis_prob = 1.0 - x_axis_prob - y_axis_prob;
      }
      
      AxisProbability(const number_t& x_p_,
                      const number_t& y_p_) {
        x_axis_prob = x_p_;
        y_axis_prob = y_p_;
        z_axis_prob = 1.0 - x_axis_prob - y_axis_prob;
      }
      number_t x_axis_prob;
      number_t y_axis_prob;
      number_t z_axis_prob;
    };

    class MatchableWorld {
    public:
      MatchableWorld();
      virtual ~MatchableWorld();

      struct Parameters {
        Parameters() {
          resolution = 1.0;
          width = 10;
          height = 10;
          num_points = 0;
          num_lines = 0;
          num_planes = 0;

          line_axis_probabilities = AxisProbability(0.3,0.3);
          plane_axis_probabilities = AxisProbability(0.15,0.15);
        }
        
        number_t resolution;
        size_t width;
        size_t height;
        size_t num_points;
        size_t num_lines;
        size_t num_planes;

        AxisProbability line_axis_probabilities;
        AxisProbability plane_axis_probabilities;
      };

      //! @brief inline get/set methods
      inline Parameters& mutableParams() {return _params;}
      inline const Parameters& params() const {return _params;}
      
      inline const MatchableSet& landmarks() const {return _landmarks;}
      inline const CellPairPlaneMap& walls() const {return _walls;}
      inline const bool& isValid() const {return _is_valid;}
      inline const size_t& numRemovedWalls() const {return _removed_walls;}

      inline bool removeWall(const CellPair& cell_motion_) {
        CellPairPlaneMap::iterator wit = _walls.find(cell_motion_);
        if (wit != _walls.end()) {
          //ia remove the landmark from the set
          _landmarks.erase(wit->second);
          _walls.erase(wit);
          ++_removed_walls;
          return true;
        }
        return false;
      }

      void createGrid();
      
    protected:
      //ia utils functions
      void _generateScatteredPoints(std::mt19937& random_generator_);
      void _generateScatteredLines(std::mt19937& random_generator_);
      void _generateScatteredPlanes(std::mt19937& random_generator_);

      //ia world inner structures
      MatchableSet _landmarks;
      CellPairPlaneMap _walls;

      //ia parameters
      Parameters _params;

      //ia stuff
      bool _is_valid;
      size_t _removed_walls;
    };
  }
}

