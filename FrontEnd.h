//
// Created by guoxt on 19-8-25.
//

#ifndef STEORO_VISUAL_ODOMETRY_FRONTEND_H
#define STEORO_VISUAL_ODOMETRY_FRONTEND_H

#include "config.h"
#include <opencv2/features2d.hpp>


#define INIT 0
#define TRACKING_GOOD 1
#define TRACKING_BAD 2
#define LOST 3

class BackEnd;

class Camera;
class Frame;
class Map;

class FrontEnd {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<FrontEnd> Ptr;

    FrontEnd();

    void AddFrame(shared_ptr<Frame> frame);
    void setMap(shared_ptr<Map> map);
    //void setBackEnd(BackEnd::Ptr backend);
    //void setViewer(ViewerPtr viewer);

    int getStatus();

    void setCameraModels(shared_ptr<Camera> left, shared_ptr<Camera> right);

private:

    int status_;

    shared_ptr<Frame> current_frame_, previous_frame_;
    shared_ptr<Camera> left_camera_, right_camera_;
    shared_ptr<Map> map_;
    shared_ptr<BackEnd> backend_;

    int current_tracking_;

    int num_;
    int num_init; //number of feature needed for initialization
    int num_tracking_high; //threshold for tracking good
    int num_tracking_low; //threshold for tracking bad (but still in track)
    int num_keyframe; //threshold for whether choosing it for keyframe

    Sophus::SE3d relative_motion;

    cv::Ptr<cv::GFTTDetector> gftt_;

    bool Track();
    bool Reset();
    int TrackLastFrame();
    int EstimateCurrentPose();
    bool InsertKeyframe();
    bool StereoInit();
    int DetectFeatures();
    int FindFeaturesInRight();
    bool BuildInitMap();
    int TriangulateNewPoints();
    void SetObservationsForKeyFrame();
    bool triangulation(const vector<Sophus::SE3d>& poses, const vector<Eigen::Vector3d> points, Eigen::Vector3d& pWorlds);



};


#endif //STEORO_VISUAL_ODOMETRY_FRONTEND_H
