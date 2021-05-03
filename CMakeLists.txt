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
find_package(Pangolin 0.5 REQUIRED)


message(STATUS "Found eigen3 lib in: " ${EIGEN3_INCLUDE_DIR})
message(STATUS "Found sophus lib in: " ${Sophus_INCLUDE_DIRS})
message(STATUS "Found opencv lib in: " ${OpenCV_INCLUDE_DIRS})
message(STATUS "Found Pangolin lib in: " ${Pangolin_INCLUDE_DIRS})

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${Sophus_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    include
)

add_library(stereo_visual_odometry_lib 
            src/io/data_reader.cc
            src/io/stereo_visual_odometry.cc
            src/types/frame.cc)
target_link_libraries(stereo_visual_odometry_lib
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${Pangolin_LIBRARIES}
)
add_executable(stereo_visual_odometry 
                src/main.cpp)
target_link_libraries(stereo_visual_odometry
                        stereo_visual_odometry_lib)


find_package(GTest REQUIRED)

include_directories(test)
add_executable(all_test test/test_all.cpp)

target_link_libraries(all_test
                        ${GTEST_BOTH_LIBRARIES}
                        pthread
                        stereo_visual_odometry_lib)

add_test(all_test all_test)
enable_testing()