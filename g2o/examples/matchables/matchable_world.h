#pragma once
#include <set>
#include <map>
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/matchables3d/types_matchables.h"

namespace g2o {
  namespace matchables {

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
    
    typedef std::map<CellPair,MatchablePtr> CellPairPlaneMap;

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

      inline const MatchablePtrSet& landmarks() const {return _landmarks;}
      inline const CellPairPlaneMap& walls() const {return _walls;}
      inline const bool& isValid() const {return _is_valid;}

      void createGrid();
      void removeWalls(int num_hits);
      
    protected:
      number_t  _resolution;
      size_t    _width;
      size_t    _height;

      MatchablePtrSet _landmarks;
      CellPairPlaneMap _walls;

      bool _is_created;
      bool _is_valid;
    };
  }
}

