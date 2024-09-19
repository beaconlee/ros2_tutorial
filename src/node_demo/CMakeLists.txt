cmake_minimum_required(VERSION 3.8)
project(node_demo)


# 设置默认编译类型为 Debug，如果没有指定
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)

  # 如果是 Debug 模式，添加调试信息 (-g)
  if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-g)
  endif()

endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(beacon src/beacon.cpp)
add_executable(twis src/twis.cc)
add_executable(sub_twis src/sub_twis.cc)
add_executable(nav_odom src/nav_odom.cc)
target_include_directories(beacon PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(beacon PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  beacon
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
)
ament_target_dependencies(
  sub_twis
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
)
ament_target_dependencies(
  twis
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
)
ament_target_dependencies(
  nav_odom
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2_ros"
)

# install(TARGETS beacon launch urdf
#   DESTINATION lib/${PROJECT_NAME})
# install(
#   DIRECTORY  launch   urdf
#   DESTINATION share/${PROJECT_NAME}
# )

install(TARGETS 
  beacon
  twis
  sub_twis
  nav_odom
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch urdf rviz DESTINATION share/${PROJECT_NAME})


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
