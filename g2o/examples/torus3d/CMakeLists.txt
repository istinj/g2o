ADD_EXECUTABLE(create_torus3d
  create_torus3d.cpp
)

SET_TARGET_PROPERTIES(create_torus3d PROPERTIES OUTPUT_NAME create_torus3d${EXE_POSTFIX})
TARGET_LINK_LIBRARIES(create_torus3d core types_slam3d)
