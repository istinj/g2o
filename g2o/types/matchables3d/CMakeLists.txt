ADD_LIBRARY(types_matchables_library ${G2O_LIB_TYPE}
            matchable.cpp                   matchable.h
            edge_se3_matchable.cpp          edge_se3_matchable.h
            vertex_matchable.cpp            vertex_matchable.h
            types_matchables.cpp            types_matchables.h
            g2o_types_matchable_api.h
            )

SET_TARGET_PROPERTIES(types_matchables_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_matchables)
TARGET_LINK_LIBRARIES(types_matchables_library 
                      types_slam3d
                      core)
#ia drawing things
IF(G2O_HAVE_OPENGL)
  TARGET_LINK_LIBRARIES(types_matchables_library opengl_helper ${OPENGL_gl_LIBRARY})
ENDIF()

