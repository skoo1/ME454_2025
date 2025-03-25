cmake_minimum_required (VERSION 3.16)
project(homework2)
set(CMAKE_CXX_STANDARD 17)

add_executable(test src/mymat.cpp src/test.cpp)

add_library(mymat SHARED src/mymat.cpp)