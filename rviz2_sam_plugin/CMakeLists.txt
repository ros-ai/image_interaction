cmake_minimum_required(VERSION 3.8)
project(rviz2_sam_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_sam_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_library(
  rviz2_sam_plugin
  SHARED
  src/sam_client.cpp
)

target_include_directories(
  rviz2_sam_plugin
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  rviz2_sam_plugin
  ros_sam_msgs
  rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS rviz2_sam_plugin
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(OpenCV REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(
    test_sam_client
    test/test_sam_client.cpp
  )

  target_include_directories(
    test_sam_client
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(
    test_sam_client
    ros_sam_msgs
    rclcpp
    cv_bridge
    OpenCV
  )

  target_link_libraries(
    test_sam_client
    rviz2_sam_plugin
  )

  install(
    DIRECTORY include/
    DESTINATION include
  )

  install(
    TARGETS test_sam_client
    DESTINATION lib/${PROJECT_NAME}
  )

endif()

ament_package()
