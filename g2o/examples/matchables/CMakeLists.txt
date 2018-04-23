#ia new graph generator
ADD_LIBRARY(matchable_generator_library ${G2O_LIB_TYPE}
  matchable_generator.h
  matchable_generator.cpp)
SET_TARGET_PROPERTIES(matchable_generator_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}matchable_generator)

TARGET_LINK_LIBRARIES(matchable_generator_library
  types_matchables_library
  types_slam3d
  core 
  )

ADD_EXECUTABLE(matchable_graph_generator matchable_graph_generator.cpp)

SET_TARGET_PROPERTIES(matchable_graph_generator PROPERTIES OUTPUT_NAME matchable_graph_generator${EXE_POSTFIX})

TARGET_LINK_LIBRARIES(matchable_graph_generator
  g2o_cli_library
  matchable_generator_library
  types_matchables_library
  types_slam3d
  )
  
  
  
  
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
  core 
  )

ADD_EXECUTABLE(matchable_simulator_3d matchable_simulator_3d.cpp)

SET_TARGET_PROPERTIES(matchable_simulator_3d PROPERTIES OUTPUT_NAME matchable_simulator_3d${EXE_POSTFIX})

TARGET_LINK_LIBRARIES(matchable_simulator_3d
  matchable_world_simulator_library
  types_matchables_library
  types_slam3d
  g2o_cli_library 
  )
