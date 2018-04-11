INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR})

ADD_LIBRARY(types_matchables_library ${G2O_LIB_TYPE}
  matchable.cpp matchable.h
  edge_se3_matchable.cpp edge_se3_matchable.h
  vertex_matchable.cpp vertex_matchable.h
  types_matchable.cpp types_matchable.h
  g2o_types_matchable_api.h
  )

SET_TARGET_PROPERTIES(types_matchables_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_matchables)

TARGET_LINK_LIBRARIES(types_matchables_library 
  types_slam3d
  core 
  )

IF(G2O_HAVE_OPENGL)
  TARGET_LINK_LIBRARIES(types_matchables_library opengl_helper ${OPENGL_gl_LIBRARY} )
ENDIF()

#ADD_EXECUTABLE(test_matchables
#  test_matchables.cpp
#)

#SET_TARGET_PROPERTIES(test_matchables PROPERTIES OUTPUT_NAME test_matchables${EXE_POSTFIX})

#TARGET_LINK_LIBRARIES(test_matchables matchables_library)

ADD_EXECUTABLE(graph_generator
  graph_generator.cpp
)

SET_TARGET_PROPERTIES(graph_generator PROPERTIES OUTPUT_NAME graph_generator${EXE_POSTFIX})

TARGET_LINK_LIBRARIES(graph_generator
  types_matchables_library
  types_slam3d
)
