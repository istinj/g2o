INCLUDE_DIRECTORIES(${CHOLMOD_INCLUDE_DIR})

# chordal comparator
ADD_EXECUTABLE(chordal_comparator chordal_comparator.cpp)
SET_TARGET_PROPERTIES(chordal_comparator PROPERTIES OUTPUT_NAME chordal_comparator${EXE_POSTFIX})
TARGET_LINK_LIBRARIES(chordal_comparator core types_slam3d solver_cholmod solver_csparse)

# chordal converter
ADD_EXECUTABLE(chordal_converter chordal_converter.cpp)
SET_TARGET_PROPERTIES(chordal_converter PROPERTIES OUTPUT_NAME chordal_converter${EXE_POSTFIX})
TARGET_LINK_LIBRARIES(chordal_converter core types_slam3d)

