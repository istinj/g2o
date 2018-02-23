ADD_EXECUTABLE(noise_adder3d
  noise_adder3d.cpp
)

SET_TARGET_PROPERTIES(noise_adder3d PROPERTIES OUTPUT_NAME noise_adder3d${EXE_POSTFIX})

TARGET_LINK_LIBRARIES(noise_adder3d types_slam3d)

