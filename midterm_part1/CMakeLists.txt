# CMakeLists to build the simulation object

cmake_minimum_required (VERSION 3.16)
project(midterm_part1)
set(CMAKE_CXX_STANDARD 17)

# Find mymat and myquaternion library
find_library(MYMAT_LIB NAMES mymat PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)
find_library(MYQUATERNION_LIB NAMES myquaternion PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)

# Per-student macro
macro(build_test arg)

  # Build simulation library
  add_library(singlebodysim_${arg} STATIC src/singlebodysim_${arg}.cpp)

  target_include_directories(singlebodysim_${arg} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/include
  )

  target_link_libraries(singlebodysim_${arg} PUBLIC
    ${MYMAT_LIB}
    ${MYQUATERNION_LIB}
  )

  add_executable(test_${arg} src/test.cpp)

  target_include_directories(test_${arg} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  target_link_libraries(test_${arg} PRIVATE singlebodysim_${arg} )

  # Ensure executable finds shared libraries at runtime
  set_target_properties(test_${arg} PROPERTIES
    BUILD_RPATH ${CMAKE_CURRENT_SOURCE_DIR}/lib
    INSTALL_RPATH ${CMAKE_CURRENT_SOURCE_DIR}/lib
  )

endmacro()

build_test(20250000) # your student id here