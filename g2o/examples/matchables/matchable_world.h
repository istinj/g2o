#pragma once
#include <set>
#include <map>
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/matchables3d/types_matchables.h"

namespace g2o {
  namespace matchables {

    //!TODO:
    //1 probabilistic axis selection in lines
    //2 0 to 1 probability for axis selection (to select a preferred axis)
    //3 remove "removeWalls" function and move this phase in the motion simulation phase

    struct CellPair{
      CellPair(const Eigen::Vector2i &first_cell_,
               const Eigen::Vector2i &second_cell_):
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

      inline bool operator ==(const CellPair &c) const {
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
    
    typedef std::map<CellPair,Matchable*> CellPairPlaneMap;

    class MatchableWorld {
    public:
      MatchableWorld();
      virtual ~MatchableWorld();

      //! @brief inline get/set methods
      inline const number_t& resolution() const {return _resolution;}
      inline void setResolution(const number_t& resolution_) {_resolution = resolution_;}
      
      inline const size_t& width() const {return _width;}
      inline void setWidth(const size_t& width_) {_width = width_;}

      inline const size_t& height() const {return _height;}
      inline void setHeight(const size_t& height_) {_height = height_;}

      inline const size_t& numPoints() const {return _num_points;}
      inline void setNumPoints(const size_t& num_points_) {_num_points = num_points_;}
      
      inline const size_t& numLines() const {return _num_lines;}
      inline void setNumLines(const size_t& num_lines_) {_num_lines = num_lines_;}
      
      inline const size_t& numPlanes() const {return _num_planes;}
      inline void setNumPlanes(const size_t& num_planes_) {_num_planes = num_planes_;}

      inline const MatchableSet& landmarks() const {return _landmarks;}
      inline const CellPairPlaneMap& walls() const {return _walls;}
      inline const bool& isValid() const {return _is_valid;}

      void createGrid();
      void removeWalls(int num_hits);
      
    protected:
      //ia utils functions
      void _generateScatteredPoints(std::mt19937& random_generator_);
      void _generateScatteredLines(std::mt19937& random_generator_);
      void _generateScatteredPlanes(std::mt19937& random_generator_);
      
      //ia world configuration
      double  _resolution;
      size_t  _width;
      size_t  _height;
      size_t  _num_points;
      size_t  _num_lines;
      size_t  _num_planes;

      //ia world inner structures
      MatchableSet _landmarks;
      CellPairPlaneMap _walls;

      //ia stuff
      bool _is_created;
      bool _is_valid;
    };
  }
}

