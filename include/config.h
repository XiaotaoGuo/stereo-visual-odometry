
#ifndef STEORO_VISUAL_ODOMETRY_CONFIG_H
#define STEORO_VISUAL_ODOMETRY_CONFIG_H

#endif //STEORO_VISUAL_ODOMETRY_CONFIG_H

// std
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <typeinfo>
#include <unordered_map>
#include <vector>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

// define the commonly included file to avoid a long include list
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>


// include self-defined class
#include "BackEnd.h"
#include "Camera.h"
#include "FrontEnd.h"
#include "Feature.h"
#include "Frame.h"
#include "Map.h"
#include "MapPoint.h"
#include "g2o_types.h"
#include "viewer.h"


using namespace std;

class Frame;
class Feature;
class Map;
class MapPoint;
class Camera;

typedef unordered_map<unsigned long, shared_ptr<MapPoint>> LandmarkType;
typedef unordered_map<unsigned long, shared_ptr<Frame>> FrameType;
typedef shared_ptr<Frame> FramePtr;
typedef shared_ptr<Map> MapPtr;
typedef shared_ptr<Camera> CamPtr;
typedef shared_ptr<Feature> FeaturePtr;
typedef shared_ptr<MapPoint> MapPointPtr;
