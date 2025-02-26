cmake_minimum_required(VERSION 3.5)
project(anscer_task)

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

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveTrajectory.srv"
  DEPENDENCIES geometry_msgs
)

# Add trajectory publisher executable
add_executable(trajectory_publisher src/trajectory_publisher.cpp)
ament_target_dependencies(trajectory_publisher
  rclcpp
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Add trajectory reader executable
add_executable(trajectory_reader src/trajectory_reader.cpp)
ament_target_dependencies(trajectory_reader
  rclcpp
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Link the generated interfaces using the modern approach
rosidl_get_typesupport_target(cpp_typesupport_target 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(trajectory_publisher 
  "${cpp_typesupport_target}")

target_link_libraries(trajectory_reader 
  "${cpp_typesupport_target}")

# Install targets
install(TARGETS
  trajectory_publisher
  trajectory_reader
  DESTINATION lib/${PROJECT_NAME}
)

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

# Install directories
install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()