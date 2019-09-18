//
// Created by guoxt on 19-8-25.
//

#ifndef STEORO_VISUAL_ODOMETRY_FRONTEND_H
#define STEORO_VISUAL_ODOMETRY_FRONTEND_H

#include "config.h"
#include "BackEnd.h"
#include <opencv2/features2d.hpp>


#define TRACKING_INIT 0
#define TRACKING_GOOD 1
#define TRACKING_BAD 2
#define LOST 3

class Camera;
class Frame;
class Map;
class Viewer;
class BackEnd;

class FrontEnd {
public:
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<FrontEnd> Ptr;

    FrontEnd();

    /// add current frame
    bool addFrame(FramePtr frame);

    /// Set main structure
    void setMap(MapPtr map);
    void setBackend(std::shared_ptr<BackEnd> backend);
    void setViewer(std::shared_ptr<Viewer> viewer);
    void setCameras(CamPtr left, CamPtr right);

    int getStatus() const;



private:

    bool Track();
    bool Reset();
    int trackLastFrame();
    int estimateCurrentPose();
    bool insertKeyFrame();
    bool StereoInit();
    int detectFeatures();
    int findFeaturesInRight();
    bool buildInitMap();
    int triangulateNewPoints();
    void setObservationsForKeyFrame();

    // data
    int status_ = TRACKING_INIT;

    FramePtr current_frame_ = nullptr;
    FramePtr last_frame_ = nullptr;
    CamPtr left_camera_ = nullptr;
    CamPtr right_camera_ = nullptr;

    MapPtr map_ = nullptr;
    shared_ptr<BackEnd> backend_ = nullptr;
    shared_ptr<Viewer> viewer_ = nullptr;

    Sophus::SE3d relative_motion_;  // estimated pose for optimazation

    int tracking_inliers_ = 0;  // inliers, used for testing new keyframes

    // params
    int num_features_ = 200;
    int num_features_init_ = 100;
    int num_features_tracking_ = 50;
    int num_features_tracking_bad_ = 20;
    int num_features_needed_for_keyframe_ = 80;

    // utilities
    cv::Ptr<cv::FeatureDetector> detector_;  // feature detector in opencv
    //cv::Ptr<cv::GFTTDetector> detector_;  // feature detector in opencv
    //cv::Ptr<cv::ORB> detector_;  // feature detector in opencv
    bool triangulation(const vector<Sophus::SE3d>& poses, const vector<Eigen::Vector3d> points, Eigen::Vector3d& pWorlds);



};


#endif //STEORO_VISUAL_ODOMETRY_FRONTEND_H
