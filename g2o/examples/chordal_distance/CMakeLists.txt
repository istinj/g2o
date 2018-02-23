INCLUDE_DIRECTORIES(${CHOLMOD_INCLUDE_DIR})

ADD_LIBRARY(chordal_distance_library ${G2O_LIB_TYPE}
  se3_chordal_ops.h
  vertex_se3_chord.h
  vertex_se3_chord.cpp
  edge_se3_chord.h
  edge_se3_chord.cpp
  types_chordal_distance.h
  types_chordal_distance.cpp
)

SET_TARGET_PROPERTIES(chordal_distance_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}chordal_distance)

TARGET_LINK_LIBRARIES(chordal_distance_library core solver_cholmod solver_csparse)
# viewer stuff
TARGET_LINK_LIBRARIES(chordal_distance_library opengl_helper ${OPENGL_gl_LIBRARY})

ADD_EXECUTABLE(chordal_comparator
  chordal_comparator.cpp
)

SET_TARGET_PROPERTIES(chordal_comparator PROPERTIES OUTPUT_NAME chordal_comparator${EXE_POSTFIX})

TARGET_LINK_LIBRARIES(chordal_comparator chordal_distance_library types_slam3d)

