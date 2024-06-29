cmake_minimum_required(VERSION 3.8)
project(beacon)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies 
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

include_directories(include)


add_executable(bumpgo
  src/planning/a_star.cc
)

add_executable(vff_avoidance
  src/vff_avoidance/avoidance_node.cc
)

install(TARGETS
  vff_avoidance
  DESTINATION lib/${PROJECT_NAME}
)

# add_library(${PROJECT_NAME} SHARED src/vff_avoidance/avoidance_node.cc)
# ament_target_dependencies(${PROJECT_NAME} ${dependencies})

ament_target_dependencies(vff_avoidance ${dependencies})
# target_link_libraries(vff_avoidance ${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
