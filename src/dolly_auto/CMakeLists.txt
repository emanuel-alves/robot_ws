cmake_minimum_required(VERSION 3.5)
project(dolly_auto)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dolly_data REQUIRED)

set(target 
  "dolly_print_locale" 
  "dolly_set_position"
  "dolly_auto"
)

###########################################################################
add_executable(dolly_print_locale src/dolly_print_locale.cpp)

ament_target_dependencies(dolly_print_locale
  "rclcpp"
  "nav_msgs"
)

add_executable(dolly_set_position src/dolly_set_position.cpp)

ament_target_dependencies(dolly_set_position
  "rclcpp"
  "nav_msgs"
  "dolly_data"
)

add_executable(dolly_auto src/dolly_auto.cpp)

ament_target_dependencies(dolly_auto
  "rclcpp"
  "geometry_msgs"
  "dolly_data"
  "nav_msgs"
)

###########################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
