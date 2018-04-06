ADD_EXECUTABLE(test_hierarchical_2
  test_hierarchical_2.cpp)

TARGET_LINK_LIBRARIES(test_hierarchical_2 g2o_cli_library types_slam3d)

SET_TARGET_PROPERTIES(test_hierarchical_2 PROPERTIES OUTPUT_NAME test_hierarchical_2${EXE_POSTFIX})

