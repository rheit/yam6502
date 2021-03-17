add_executable(test test.cpp "test.h" "simple.cpp" "minic64.cpp")
target_link_libraries(test yam6502)
target_compile_definitions(test PUBLIC TESTS_BIN="${CMAKE_CURRENT_SOURCE_DIR}/bin")
