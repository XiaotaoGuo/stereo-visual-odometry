cmake_minimum_required(VERSION 2.8)

# set(CMAKE_CXX_FLAGS "-03")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(steoro_visual_odometry)


set(CMAKE_BUILD_TYPE  "Release")

# add cmake module configuration
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules/g2o)
#list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules/ceres)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenCV 3.0 REQUIRED)
find_package(Pangolin REQUIRED)

message(STATUS "Found eigen3 lib in: " ${EIGEN3_INCLUDE_DIR})
message(STATUS "Found sophus lib in: " ${Sophus_INCLUDE_DIRS})
message(STATUS "Found opencv lib in: " ${OpenCV_INCLUDE_DIRS})
message(STATUS "Found pangolin lib in: " ${Pangolin_INCLUDE_DIRS})

find_package(G2O REQUIRED)
find_package(Cholmod REQUIRED)
message(STATUS "Found g2o lib in: " ${G2O_INCLUDE_DIR})
message(STATUS "Found cholmod lib in: " ${CHOLMOD_INCLUDE_DIR})

include_directories(${EIGEN3_INCLUDE_DIR}
                    ${Sophus_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${Pangolin_INCLUDE_DIRS}
                    ${G2O_INCLUDE_DIR}
                    ${CHOLMOD_INCLUDE_DIR}
                    include)

# for g2o, we manually set its libs
set(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear
        g2o_types_icp g2o_types_slam2d g2o_types_sba g2o_types_slam3d g2o_core g2o_interface
        g2o_solver_csparse g2o_solver_structure_only g2o_csparse_extension g2o_opengl_helper g2o_solver_dense
        g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse)

add_executable(vo vo.cpp
        src/FrontEnd.cpp
        src/BackEnd.cpp
        src/Feature.cpp
        src/Frame.cpp
        src/Map.cpp
        src/MapPoint.cpp
        src/Camera.cpp
        src/viewer.cpp
        src/Visual_odometry.cpp)

target_link_libraries(vo
        ${G2O_LIBS}
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES})