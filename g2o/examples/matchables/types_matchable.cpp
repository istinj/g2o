#include "types_matchable.h"
#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"
#include <iostream>

namespace g2o {
  namespace matchables {
    
    G2O_REGISTER_TYPE_GROUP(matchable);

    G2O_REGISTER_TYPE(VERTEX_MATCHABLE, VertexMatchable);
    G2O_REGISTER_TYPE(EDGE_SE3_MATCHABLE, EdgeSE3Matchable);

#ifdef G2O_HAVE_OPENGL
    G2O_REGISTER_ACTION(VertexMatchableDrawAction);
    G2O_REGISTER_ACTION(EdgeSE3MatchableDrawAction);
#endif

  }
}
