ADD_LIBRARY(g2o_hierarchical_online_library ${G2O_LIB_TYPE}
  local_star.h
  local_star.cpp
#   edge_creator.cpp
#   star.cpp
#   edge_types_cost_function.cpp
#   backbone_tree_action.cpp
#   simple_star_ops.cpp
#   g2o_hierarchical_test_functions.cpp
#   edge_labeler.h
#   edge_creator.h
#   star.h
#   edge_types_cost_function.h
#   backbone_tree_action.h
#   simple_star_ops.h
#   g2o_hierarchical_api.h
)

# library
SET_TARGET_PROPERTIES(g2o_hierarchical_online_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}hierarchical_online)
TARGET_LINK_LIBRARIES(g2o_hierarchical_online_library core)

# test application
ADD_EXECUTABLE(g2o_hierarchical_online_app g2o_hierachical_online.cpp)
TARGET_LINK_LIBRARIES(g2o_hierarchical_online_app g2o_cli_library g2o_hierarchical_online_library types_slam3d)
SET_TARGET_PROPERTIES(g2o_hierarchical_online_app PROPERTIES OUTPUT_NAME g2o_hierarchical_online${EXE_POSTFIX})

