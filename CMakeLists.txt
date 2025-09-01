cmake_minimum_required(VERSION 3.8)
project(rerun_viz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rerun_sdk_vendor REQUIRED)
find_package(rerun_sdk REQUIRED)
find_package(sensor_msgs REQUIRED)

# Declare a static lib to contain most of the business logic
add_library(
  rerun_viz_lib
  STATIC
  src/msg_conversion/sensor_msgs/point_cloud2.cpp
)
target_include_directories(rerun_viz_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(rerun_viz_lib sensor_msgs)

# Declare the actual node executable
add_executable(rerun_viz src/rerun_viz.cpp)

target_include_directories(rerun_viz PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(rerun_viz PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(rerun_viz rerun_sdk rerun_viz_lib)
ament_target_dependencies(rerun_viz ament_index_cpp)

install(TARGETS rerun_viz
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()


  find_package(ament_cmake_gtest)
  ament_add_gtest(
    unit_tests
    test/test_simple.cpp
    test/msg_conversion/sensor_msgs/test_point_cloud2.cpp
  )

  target_link_libraries(unit_tests rerun_viz_lib)

  # find_package(ament_cmake_gmock REQUIRED)
  # ament_add_gmock(some_test <test_sources>)

endif()

ament_package()
