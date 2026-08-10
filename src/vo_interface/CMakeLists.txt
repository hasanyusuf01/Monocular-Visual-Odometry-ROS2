cmake_minimum_required(VERSION 3.8)
project(vo_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find the required build packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 2. Generate the custom messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FlowStats.msg"
  "srv/ResetOdom.srv"
  DEPENDENCIES std_msgs
)

# 3. Standard ROS2 testing block (This is where your error was!)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 4. Finalize the package
ament_package()