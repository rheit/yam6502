add_library(yam6502 yam6502.cpp "${CMAKE_SOURCE_DIR}/include/yam6502.hpp")
target_include_directories(yam6502 PUBLIC ../include)
target_compile_features(yam6502 PUBLIC cxx_std_20)

