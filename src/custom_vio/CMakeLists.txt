cmake_minimum_required(VERSION 3.8)
project(custom_vio)

# Default to C++17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==============================================================================
# 1. FIND DEPENDENCIES
# ==============================================================================

# ROS 2 Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
# target_link_libraries(vio_node )

# External System Libraries
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(cnpy REQUIRED)

# ==============================================================================
# 2. INCLUDE DIRECTORIES
# ==============================================================================

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
)

# ==============================================================================
# 3. DECLARE EXECUTABLE TARGETS
# ==============================================================================

# Target 1: VO Front-End Node
add_executable(vo_frontend_node
  src/vo_frontend_node.cpp
)

# Link ROS 2 dependencies to Front-End
ament_target_dependencies(vo_frontend_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  std_msgs
  ament_index_cpp
  image_transport
)

# Link non-ROS C++ libraries (OpenCV)
target_link_libraries(vo_frontend_node
  ${OpenCV_LIBS}
  yaml-cpp::yaml-cpp
)


# Target 2: ESKF Back-End Node
add_executable(eskf_backend_node
  src/eskf_backend_node.cpp
)

# Link ROS 2 dependencies to Back-End
ament_target_dependencies(eskf_backend_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

# Link non-ROS C++ libraries (Eigen3)
target_link_libraries(eskf_backend_node
  Eigen3::Eigen
)

# ==============================================================================
# 4. INSTALLATION RULES
# ==============================================================================

# Install executables to lib/custom_vio
install(TARGETS
  vo_frontend_node
  eskf_backend_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files, launch files, and config directories if they exist
install(DIRECTORY
  include/
  launch
  config
  metrics
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h" PATTERN "*.py" PATTERN "*.yaml"
)

# ==============================================================================
# 5. AMENT PACKAGE HOOK (MUST BE AT THE VERY END)
# ==============================================================================

ament_package()