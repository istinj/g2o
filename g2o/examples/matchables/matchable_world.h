#pragma once
#include <set>
#include <map>
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/matchables3d/types_matchables.h"

namespace g2o {
  namespace matchables {
    class MatchableWorld {
    public:
      MatchableWorld();
      virtual ~MatchableWorld();

      //! @brief inline get/set methods
      inline const number_t& resolution() const {return _resolution;}
      inline void setResolution(const number_t& resolution_) {_resolution = resolution_;}
      
      inline int width() const {return _width;}
      inline void setWidth(const int width_) {_width = width_;}

      inline int height() const {return _height;}
      inline void setHeight(const int height_) {_height = height_;}
      
    protected:
      number_t  _resolution;
      int       _width;
      int       _height;
      
    };
  }
}

