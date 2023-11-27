cmake_minimum_required(VERSION 3.8)
project(jp200_demo_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(jp200_msgs REQUIRED)

include_directories(./include/)

add_library(jp200_demo_component SHARED
  src/component.cpp
)
ament_target_dependencies(jp200_demo_component
  rclcpp
  std_msgs
  jp200_msgs
  rclcpp_components
)

ament_export_libraries(jp200_demo_component)
rclcpp_components_register_nodes(jp200_demo_component "jp200_demo_component::JP200DemoComp")

add_executable(jp200_demo_node src/node.cpp)
target_link_libraries(jp200_demo_node jp200_demo_component)
ament_target_dependencies(jp200_demo_node
  rclcpp
  std_msgs
  jp200_msgs
  rclcpp_components
)

install(
  DIRECTORY "include/"
  DESTINATION include
)
install(TARGETS
  jp200_demo_node
  DESTINATION lib/jp200_demo
)
install(TARGETS
  jp200_demo_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

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
