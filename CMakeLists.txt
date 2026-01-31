cmake_minimum_required(VERSION 3.16) # Increased for Noble/Jazzy compatibility
project(orbslam3)

set(ORB_SLAM3_DIR "/opt/ORB_SLAM3")

# 2. Add all necessary include paths for ORB-SLAM3 and its Thirdparty deps
include_directories(
  include
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
)

# 1. Jazzy/Noble requires C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 2. Add compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# 3. Standard ROS 2 Jazzy dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)

# 4. External dependencies
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED CONFIG)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# If ORB_SLAM3 is a separate library you built, ensure its path is set
# find_package(ORB_SLAM3 REQUIRED)

# 5. Group common dependencies to simplify target linking
set(ORB_SLAM3_DEPS
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  cv_bridge::cv_bridge
  message_filters::message_filters
  image_transport::image_transport
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
  Pangolin::Pangolin
  # Add ORB_SLAM3_LIBRARIES here if found via find_package
)

# 6. Include Directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# --- Macro to reduce boilerplate for the nodes ---
macro(add_orbslam3_node NAME)
    add_executable(${NAME} ${ARGN})

    # 1. Link standard C++ libraries
    target_link_libraries(${NAME}
    ${OpenCV_LIBRARIES}
    Eigen3::Eigen
    ${Pangolin_LIBRARIES}

    ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
  )

    # 2. Let ROS handle all ROS-specific linking
    # Note: Use the package names, NOT the :: namespace names here
    ament_target_dependencies(${NAME}
    rclcpp
    sensor_msgs
    cv_bridge
    message_filters
    image_transport
    tf2_ros
    tf2_geometry_msgs
  )
endmacro()

# 7. Nodes
add_orbslam3_node(mono src/monocular/mono.cpp src/monocular/monocular-slam-node.cpp)
add_orbslam3_node(rgbd src/rgbd/rgbd.cpp src/rgbd/rgbd-slam-node.cpp)
add_orbslam3_node(stereo src/stereo/stereo.cpp src/stereo/stereo-slam-node.cpp)
add_orbslam3_node(stereo-inertial src/stereo-inertial/stereo-inertial.cpp src/stereo-inertial/stereo-inertial-node.cpp)

# 8. Installation
install(TARGETS
    mono
    rgbd
    stereo
    stereo-inertial
  DESTINATION lib/${PROJECT_NAME})

# Install config and vocabulary
install(DIRECTORY config vocabulary
  DESTINATION share/${PROJECT_NAME})

ament_package()
