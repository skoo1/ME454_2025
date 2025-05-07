cmake_minimum_required (VERSION 3.16)
project(four_bar_linkage_cpp)
set(CMAKE_CXX_STANDARD 17)

include_directories(include)
find_library(MYMAT_LIB NAMES mymat PATHS "${CMAKE_CURRENT_SOURCE_DIR}/lib")
find_package(Eigen3 REQUIRED)


macro(macro1 arg)
  add_executable(four_bar_linkage_simulation_${arg} src/four_bar_linkage_simulation_${arg}.cpp)
  target_link_libraries (four_bar_linkage_simulation_${arg} Eigen3::Eigen)
endmacro()

macro1(20250001)
