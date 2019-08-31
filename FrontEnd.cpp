//
// Created by guoxt on 19-8-25.
//

#include "FrontEnd.h"
#include <opencv2/opencv.hpp>

FrontEnd::FrontEnd() {
    gftt_ =
            cv::GFTTDetector::create(100, 0.01, 20);
    num_features_init_ = 50;
    num_features_ = 100;
}

bool FrontEnd::AddFrame(FramePtr frame) {
    current_frame_ = frame;

    switch (status_) {
        case TRACKING_INIT:
            StereoInit();
            break;
        case TRACKING_GOOD:
        case TRACKING_BAD:
            Track();
            break;
        case LOST:
            Reset();
            break;
    }

    last_frame_ = current_frame_;
    return true;
}

bool FrontEnd::Track() {
    if (last_frame_) {
        current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
    }

    int num_track_last = TrackLastFrame();
    tracking_inliers_ = EstimateCurrentPose();

    if (tracking_inliers_ > num_features_tracking_) {
        // tracking good
        status_ = TRACKING_GOOD;
    } else if (tracking_inliers_ > num_features_tracking_bad_) {
        // tracking bad
        status_ = TRACKING_BAD;
    } else {
        // lost
        status_ = LOST;
    }

    InsertKeyframe();
    relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

    if (viewer_) viewer_->AddCurrentFrame(current_frame_);
    return true;
}

bool FrontEnd::InsertKeyframe() {
    if (tracking_inliers_ >= num_features_needed_for_keyframe_) {
        // still have enough features, don't insert keyframe
        return false;
    }
    // current frame is a new keyframe
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);

    cout << "Set frame " << current_frame_->id_ << " as keyframe "
              << current_frame_->keyframe_id_ << endl;

    SetObservationsForKeyFrame();
    DetectFeatures();  // detect new features

    // track in right image
    FindFeaturesInRight();
    // triangulate map points
    TriangulateNewPoints();
    // update backend because we have a new keyframe
    backend_->updateMap();

    if (viewer_) viewer_->UpdateMap();

    return true;
}

void FrontEnd::SetObservationsForKeyFrame() {
    for (auto &feat : current_frame_->left_features_) {
        auto mp = feat->map_point_.lock();
        if (mp) mp->AddObservation(feat);
    }
}

int FrontEnd::TriangulateNewPoints() {
    std::vector<Sophus::SE3d> poses{camera_left_->pose(), camera_right_->pose()};
    Sophus::SE3d current_pose_Twc = current_frame_->Pose().inverse();
    int cnt_triangulated_pts = 0;
    for (size_t i = 0; i < current_frame_->left_features_.size(); ++i) {
        if (current_frame_->left_features_[i]->map_point_.expired() &&
            current_frame_->right_features_[i] != nullptr) {
            // 左图的特征点未关联地图点且存在右图匹配点，尝试三角化
            std::vector<Eigen::Vector3d> points{
                    camera_left_->pixel2camera(
                            Eigen::Vector2d(current_frame_->left_features_[i]->position_.pt.x,
                                 current_frame_->left_features_[i]->position_.pt.y)),
                    camera_right_->pixel2camera(
                            Eigen::Vector2d(current_frame_->right_features_[i]->position_.pt.x,
                                 current_frame_->right_features_[i]->position_.pt.y))};
            Eigen::Vector3d pworld = Eigen::Vector3d::Zero();

            if (triangulation(poses, points, pworld) && pworld[2] > 0) {
                auto new_map_point = MapPoint::CreateNewMappoint();
                pworld = current_pose_Twc * pworld;
                new_map_point->setPos(pworld);
                new_map_point->AddObservation(
                        current_frame_->left_features_[i]);
                new_map_point->AddObservation(
                        current_frame_->right_features_[i]);

                current_frame_->left_features_[i]->map_point_ = new_map_point;
                current_frame_->right_features_[i]->map_point_ = new_map_point;
                map_->InsertMapPoint(new_map_point);
                cnt_triangulated_pts++;
            }
        }
    }
    cout << "new landmarks: " << cnt_triangulated_pts << endl;
    return cnt_triangulated_pts;
}

int FrontEnd::EstimateCurrentPose() {
    // setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
            LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<BlockSolverType>(
                    g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // vertex
    VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
    vertex_pose->setId(0);
    vertex_pose->setEstimate(current_frame_->Pose());
    optimizer.addVertex(vertex_pose);

    // K
    Eigen::Matrix3d K = camera_left_->K();

    // edges
    int index = 1;
    std::vector<EdgeProjectionPoseOnly *> edges;
    std::vector<Feature::Ptr> features;
    for (size_t i = 0; i < current_frame_->left_features_.size(); ++i) {
        auto mp = current_frame_->left_features_[i]->map_point_.lock();
        if (mp) {
            features.push_back(current_frame_->left_features_[i]);
            EdgeProjectionPoseOnly *edge =
                    new EdgeProjectionPoseOnly(mp->pos_, K);
            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            cv::Point2f pt = current_frame_->left_features_[i]->position_.pt;
            Eigen::Vector2d pt_(pt.x, pt.y);
            edge->setMeasurement(pt_);
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            edges.push_back(edge);
            optimizer.addEdge(edge);
            index++;
        }
    }

    // estimate the Pose the determine the outliers
    const double chi2_th = 5.991;
    int cnt_outlier = 0;
    for (int iteration = 0; iteration < 4; ++iteration) {
        vertex_pose->setEstimate(current_frame_->Pose());
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        cnt_outlier = 0;

        // count the outliers
        for (size_t i = 0; i < edges.size(); ++i) {
            auto e = edges[i];
            if (features[i]->is_outlier) {
                e->computeError();
            }
            if (e->chi2() > chi2_th) {
                features[i]->is_outlier = true;
                e->setLevel(1);
                cnt_outlier++;
            } else {
                features[i]->is_outlier = false;
                e->setLevel(0);
            };

            if (iteration == 2) {
                e->setRobustKernel(nullptr);
            }
        }
    }

    cout << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
              << features.size() - cnt_outlier << endl;
    // Set pose and outlier
    current_frame_->SetPose(vertex_pose->estimate());

    cout << "Current Pose = \n" << current_frame_->Pose().matrix() << endl;

    for (auto &feat : features) {
        if (feat->is_outlier) {
            feat->map_point_.reset();
            feat->is_outlier = false;  // maybe we can still use it in future
        }
    }
    return features.size() - cnt_outlier;
}

int FrontEnd::TrackLastFrame() {
    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> kps_last, kps_current;
    for (auto &kp : last_frame_->left_features_) {
        if (kp->map_point_.lock()) {
            // use project point
            auto mp = kp->map_point_.lock();
            auto px =
                    camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(cv::Point2f(px[0], px[1]));
        } else {
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(kp->position_.pt);
        }
    }

    std::vector<uchar> status;
    cv::Mat error;
    cv::calcOpticalFlowPyrLK(
            last_frame_->left_img_, current_frame_->left_img_, kps_last,
            kps_current, status, error, cv::Size(11, 11), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                             0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;

    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_current[i], 7);
            Feature::Ptr feature(new Feature(current_frame_, kp));
            feature->map_point_ = last_frame_->left_features_[i]->map_point_;
            current_frame_->left_features_.push_back(feature);
            num_good_pts++;
        }
    }

    cout << "Find " << num_good_pts << "goot map points in the last image." << endl;
    return num_good_pts;
}

bool FrontEnd::StereoInit() {
    int num_features_left = DetectFeatures();
    int num_coor_features = FindFeaturesInRight();
    if (num_coor_features < num_features_init_) {
        return false;
    }

    bool build_map_success = BuildInitMap();
    if (build_map_success) {
        status_ = TRACKING_GOOD;
        if (viewer_) {
            viewer_->AddCurrentFrame(current_frame_);
            viewer_->UpdateMap();
        }
        return true;
    }
    return false;
}

int FrontEnd::DetectFeatures() {
    cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
    for (auto &feat : current_frame_->left_features_) {
        cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                      feat->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED);
    }

    std::vector<cv::KeyPoint> keypoints;
    gftt_->detect(current_frame_->left_img_, keypoints, mask);
    int cnt_detected = 0;
    for (auto &kp : keypoints) {
        current_frame_->left_features_.push_back(
                Feature::Ptr(new Feature(current_frame_, kp)));
        cnt_detected++;
    }

    cout << "Detect " << cnt_detected << " new features" << endl;
    return cnt_detected;
}

int FrontEnd::FindFeaturesInRight() {
    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> kps_left, kps_right;
    for (auto &kp : current_frame_->left_features_) {
        kps_left.push_back(kp->position_.pt);
        auto mp = kp->map_point_.lock();
        if (mp) {
            // use projected points as initial guess
            auto px =
                    camera_right_->world2pixel(mp->pos_, current_frame_->Pose());
            kps_right.push_back(cv::Point2f(px[0], px[1]));
        } else {
            // use same pixel in left iamge
            kps_right.push_back(kp->position_.pt);
        }
    }

    std::vector<uchar> status;
    cv::Mat error;
    cv::calcOpticalFlowPyrLK(
            current_frame_->left_img_, current_frame_->right_img_, kps_left,
            kps_right, status, error, cv::Size(11, 11), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                             0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_right[i], 7);
            Feature::Ptr feat(new Feature(current_frame_, kp));
            feat->is_on_left_image_ = false;
            current_frame_->right_features_.push_back(feat);
            num_good_pts++;
        } else {
            current_frame_->right_features_.push_back(nullptr);
        }
    }
    cout << "Find " << num_good_pts << " in the right image." << endl;
    return num_good_pts;
}

bool FrontEnd::BuildInitMap() {
    std::vector<Sophus::SE3d> poses{camera_left_->pose(), camera_right_->pose()};
    size_t cnt_init_landmarks = 0;
    for (size_t i = 0; i < current_frame_->left_features_.size(); ++i) {
        if (current_frame_->right_features_[i] == nullptr) continue;
        // create map point from triangulation
        std::vector<Eigen::Vector3d> points{
                camera_left_->pixel2camera(
                        Eigen::Vector2d(current_frame_->left_features_[i]->position_.pt.x,
                             current_frame_->left_features_[i]->position_.pt.y)),
                camera_right_->pixel2camera(
                        Eigen::Vector2d(current_frame_->right_features_[i]->position_.pt.x,
                             current_frame_->right_features_[i]->position_.pt.y))};
        Eigen::Vector3d pworld = Eigen::Vector3d::Zero();

        if (triangulation(poses, points, pworld) && pworld[2] > 0) {
            auto new_map_point = MapPoint::CreateNewMappoint();
            new_map_point->setPos(pworld);
            new_map_point->AddObservation(current_frame_->left_features_[i]);
            new_map_point->AddObservation(current_frame_->right_features_[i]);
            current_frame_->left_features_[i]->map_point_ = new_map_point;
            current_frame_->right_features_[i]->map_point_ = new_map_point;
            cnt_init_landmarks++;
            map_->InsertMapPoint(new_map_point);
        }
    }
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);
    backend_->updateMap();

    cout << "Initial map created with " << cnt_init_landmarks
              << " map points" << endl;

    return true;
}

bool FrontEnd::Reset() {
    cout << "Reset is not implemented. " << endl;
    return true;
}

bool FrontEnd::triangulation(const vector<Sophus::SE3d> &poses, const vector<Eigen::Vector3d> points,
                             Eigen::Vector3d &pWorlds) {
    Eigen::MatrixXd A(2 * poses.size(), 4);
    Eigen::VectorXd b(2 * poses.size());
    b.setZero();
    for (size_t i = 0; i < poses.size(); ++i) {
        Eigen::Matrix<double, 3, 4> m = poses[i].matrix3x4();
        A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
        A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
    }
    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    pWorlds = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
        // 解质量不好，放弃
        return true;
    }
    return false;
}

