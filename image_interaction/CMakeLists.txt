cmake_minimum_required(VERSION 3.8)
project(image_interaction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)

set(image_interaction_dependencies
  rclcpp
  rviz_common
  pluginlib
)

add_library(
  image_interaction
  SHARED
  src/interaction_panel.cpp
)

target_include_directories(
  image_interaction
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  image_interaction
  ${image_interaction_dependencies}
)


pluginlib_export_plugin_description_file(rviz_common image_interaction.xml)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${image_interaction_dependencies})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS image_interaction
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # find_package(ament_cmake_gtest REQUIRED)
  # find_package(cv_bridge REQUIRED)
  # find_package(OpenCV REQUIRED)
  # # the following line skips the linter which checks for copyrights
  # # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # # the following line skips cpplint (only works in a git repo)
  # # comment the line when this package is in a git repo and when
  # # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  # ament_add_gtest(
  #   test_sam_client
  #   test/test_sam_client.cpp
  # )

  # target_include_directories(
  #   test_sam_client
  #   PRIVATE
  #   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  #   $<INSTALL_INTERFACE:include>
  # )

  # ament_target_dependencies(
  #   test_sam_client
  #   ros_sam_msgs
  #   rclcpp
  #   cv_bridge
  #   OpenCV
  # )

  # target_link_libraries(
  #   test_sam_client
  #   image_interaction
  # )

  # install(
  #   DIRECTORY include/
  #   DESTINATION include
  # )

  # install(
  #   TARGETS test_sam_client
  #   DESTINATION lib/${PROJECT_NAME}
  # )
endif()

ament_package()
