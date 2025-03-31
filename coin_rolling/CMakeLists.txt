cmake_minimum_required(VERSION 3.5)
project(coin_rolling)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(coin_rolling_msg REQUIRED)


add_executable(coin_subscriber src/coin_subscriber.cpp)
add_executable(inclined_coin_pubsub src/inclined_coin_pubsub.cpp)
add_executable(coin_pubsub src/coin_pubsub.cpp)
add_executable(test_subscriber src/test_subscriber.cpp)

ament_target_dependencies(coin_subscriber rclcpp std_msgs gazebo_msgs geometry_msgs)
ament_target_dependencies(inclined_coin_pubsub rclcpp std_msgs gazebo_msgs geometry_msgs coin_rolling_msg)
ament_target_dependencies(coin_pubsub rclcpp std_msgs gazebo_msgs geometry_msgs)
ament_target_dependencies(test_subscriber rclcpp std_msgs gazebo_msgs geometry_msgs coin_rolling_msg)



install(DIRECTORY
  launch
  rsc
  worlds
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  coin_subscriber
  coin_pubsub
  inclined_coin_pubsub
  test_subscriber
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
