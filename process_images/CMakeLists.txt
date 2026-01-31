cmake_minimum_required(VERSION 3.0)
project(process_images)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")

find_package(OpenCV 4.0 REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/../orbslam3
    ${PROJECT_SOURCE_DIR}/../orbslam3/include
    ${PROJECT_SOURCE_DIR}/../orbslam3/Thirdparty/Sophus
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(process_images 
    process_images.cpp
    ../orbslam3/src/ORBextractor.cc
)

target_link_libraries(process_images ${OpenCV_LIBS})
