cmake_minimum_required(VERSION 3.16)
project(yam6502)

if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
	string(REPLACE "/Ob1" "/Ob2" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
endif()

add_subdirectory(src)
add_subdirectory(tests)
add_subdirectory(example)

