cmake_minimum_required(VERSION 3.5.0)
project(stereo_visual_odometry VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE  -pthread)

set(CMAKE_BUILD_TYPE  "Debug")

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenCV 3.0 REQUIRED)
find_package(Boost REQUIRED filesystem)

message(STATUS "Found eigen3 lib in: " ${EIGEN3_INCLUDE_DIR})
message(STATUS "Found sophus lib in: " ${Sophus_INCLUDE_DIRS})
message(STATUS "Found opencv lib in: " ${OpenCV_INCLUDE_DIRS})

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${Sophus_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    include
)

add_library(stereo_visual_odometry_lib 
            src/io/data_reader.cc)
target_link_libraries(stereo_visual_odometry_lib
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
)
add_executable(stereo_visual_odmetry src/main.cpp)

find_package(GTest REQUIRED)

add_executable(image_reader_test test/test_image_reader.cpp)

target_link_libraries(image_reader_test
                        ${GTEST_BOTH_LIBRARIES}
                        pthread
                        stereo_visual_odometry_lib)

add_test(image_reader_test image_reader_test)
enable_testing()