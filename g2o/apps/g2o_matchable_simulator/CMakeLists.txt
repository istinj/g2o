#ia grid world simulator
ADD_LIBRARY(matchable_world_simulator_library ${G2O_LIB_TYPE}
  matchable_world.h
  matchable_world.cpp
  matchable_world_simulator.h
  matchable_world_simulator.cpp)
SET_TARGET_PROPERTIES(matchable_world_simulator_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}matchable_world_simulator)
TARGET_LINK_LIBRARIES(matchable_world_simulator_library
  types_matchables_library
  types_slam3d
  core)

ADD_EXECUTABLE(g2o_matchable_simulator3d matchable_simulator_3d.cpp)
SET_TARGET_PROPERTIES(g2o_matchable_simulator3d PROPERTIES OUTPUT_NAME g2o_matchable_simulator3d${EXE_POSTFIX})
TARGET_LINK_LIBRARIES(g2o_matchable_simulator3d
  matchable_world_simulator_library
  types_matchables_library
  types_slam3d
  g2o_cli_library)


#ia noise adder
ADD_EXECUTABLE(g2o_matchable_noise_adder g2o_matchable_noise_adder.cpp)
SET_TARGET_PROPERTIES(g2o_matchable_noise_adder PROPERTIES OUTPUT_NAME g2o_matchable_noise_adder${EXE_POSTFIX})
TARGET_LINK_LIBRARIES(g2o_matchable_noise_adder
  matchable_world_simulator_library
  types_matchables_library
  types_slam3d
  g2o_cli_library)


#ia test analytic jacobians
