INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR})

ADD_LIBRARY(matchables_library ${G2O_LIB_TYPE}
  matchable.cpp matchable.h
  edge_se3_matchable.cpp edge_se3_matchable.h
  vertex_matchable.cpp vertex_matchable.h
  types_matchable.cpp types_matchable.h
  graph_generator.cpp graph_generator.h
)

SET_TARGET_PROPERTIES(matchables_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}matchables)

TARGET_LINK_LIBRARIES(matchables_library 
  core 
  solver_csparse
  solver_cholmod
)

#ADD_EXECUTABLE(test_matchables
#  test_matchables.cpp
#)

#SET_TARGET_PROPERTIES(test_matchables PROPERTIES OUTPUT_NAME test_matchables${EXE_POSTFIX})

#TARGET_LINK_LIBRARIES(test_matchables matchables_library)
