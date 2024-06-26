cmake_minimum_required(VERSION 3.8)
project(basics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies 
    rclcpp
    std_msgs
)

add_executable(basics src/basics.cc)
# target_include_directories(basics PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
target_compile_features(basics PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(basics ${dependencies})

add_executable(publisher_basics src/publisher_basics.cc)
# target_include_directories(basics PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
target_compile_features(publisher_basics PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(publisher_basics ${dependencies})

add_executable(subscriber_basics src/subscriber_basics.cc)
# target_include_directories(basics PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
target_compile_features(subscriber_basics PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(subscriber_basics ${dependencies})


add_executable(param_basics src/param_basics.cc)
# target_include_directories(basics PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
target_compile_features(param_basics PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(param_basics ${dependencies})


add_executable(executors src/executors.cc)
# target_include_directories(basics PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
target_compile_features(executors PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(executors ${dependencies})

install(TARGETS 
  basics
  publisher_basics
  subscriber_basics
  param_basics
  executors
  DESTINATION lib/${PROJECT_NAME})


# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# beacon 可以把这个 install 和上面的 install 合并到一起
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})



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
