#include "types_chordal_distance.h"

#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

#include <iostream>

namespace g2o {
  namespace chordal {
    G2O_REGISTER_TYPE_GROUP(chordal_distance);

    G2O_REGISTER_TYPE(VERTEX_SE3:CHORD, VertexSE3Chord);
    G2O_REGISTER_TYPE(EDGE_SE3:CHORD, EdgeSE3Chord);

    // ACTIONS
    G2O_REGISTER_ACTION(VertexSE3ChordDrawAction);
    G2O_REGISTER_ACTION(EdgeSE3ChordDrawAction);
  }
}