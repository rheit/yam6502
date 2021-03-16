file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/include/*.h")
add_library(yam6502 yam6502.cpp ${HEADER_LIST})
target_include_directories(yam6502 PUBLIC ../include)
target_compile_features(yam6502 PUBLIC cxx_std_20)

