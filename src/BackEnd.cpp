#include "BackEnd.h"
#include "Map.h"

BackEnd::BackEnd() {
    backend_running_.store(true);
    backend_thread_ = thread(bind(&BackEnd::BackEndLoop, this));
}

void BackEnd::setCameras(CamPtr left, CamPtr right) {
    cam_left_ = left;
    cam_right_ = right;
}
void BackEnd::setMap(MapPtr map) {
    map_ = map;
}

void BackEnd::updateMap() {
    unique_lock<mutex> lock(data_mutex_);
    map_update_.notify_one();
}

void BackEnd::stop() {
    backend_running_.store(false);
    map_update_.notify_one();
    backend_thread_.join();
}

void BackEnd::BackEndLoop() {
    while (backend_running_.load()) {
        unique_lock<mutex> lock(data_mutex_);
        map_update_.wait(lock);

        /// 后端仅优化激活的Frames和Landmarks
        FrameType active_kfs = map_->getActiveKeyFrames();
        LandmarkType active_landmarks = map_->getActiveMapPoints();
        Optimize(active_kfs, active_landmarks);
    }
}

void BackEnd::Optimize(FrameType &keyframes,
                       LandmarkType &landmarks) {
    // setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>
            LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<BlockSolverType>(
                    g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // pose 顶点，使用Keyframe id
    map<unsigned long, VertexPose *> vertices;
    unsigned long max_kf_id = 0;
    for (auto &keyframe : keyframes) {
        auto kf = keyframe.second;
        VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
        vertex_pose->setId(kf->keyframe_id_);
        vertex_pose->setEstimate(kf->Pose());
        optimizer.addVertex(vertex_pose);
        if (kf->keyframe_id_ > max_kf_id) {
            max_kf_id = kf->keyframe_id_;
        }

        vertices.insert({kf->keyframe_id_, vertex_pose});
    }


    // 路标顶点，使用路标id索引
    map<unsigned long, VertexXYZ *> vertices_landmarks;

    // K 和左右外参
    Eigen::Matrix3d K = cam_left_->K();
    Sophus::SE3d left_ext = cam_left_->pose();
    Sophus::SE3d right_ext = cam_right_->pose();

    // edges
    int index = 1;
    double chi2_th = 5.991;  // robust kernel 阈值
    map<EdgeProjection *, FeaturePtr> edges_and_features;

    for (auto &landmark : landmarks) {
        if (landmark.second->is_outlier_) continue;
        unsigned long landmark_id = landmark.second->id_;
        auto observations = landmark.second->getObservations();
        for (auto &obs : observations) {
            if (obs.lock() == nullptr) continue;
            auto feat = obs.lock();
            if (feat->is_outlier || feat->frame_.lock() == nullptr) continue;

            auto frame = feat->frame_.lock();
            EdgeProjection *edge = nullptr;
            if (feat->is_on_left_image_) {
                edge = new EdgeProjection(K, left_ext);
            } else {
                edge = new EdgeProjection(K, right_ext);
            }

            // 如果landmark还没有被加入优化，则新加一个顶点
            if (vertices_landmarks.find(landmark_id) ==
                vertices_landmarks.end()) {
                VertexXYZ *v = new VertexXYZ;
                v->setEstimate(landmark.second->Pos());
                v->setId(landmark_id + max_kf_id + 1);
                v->setMarginalized(true);
                vertices_landmarks.insert({landmark_id, v});
                optimizer.addVertex(v);
            }

            edge->setId(index);
            edge->setVertex(0, vertices.at(frame->keyframe_id_));    // pose
            edge->setVertex(1, vertices_landmarks.at(landmark_id));  // landmark
            cv::Point2d pt(feat->position_.pt);
            Eigen::Vector2d pt_(pt.x, pt.y);
            edge->setMeasurement(pt_);
            edge->setInformation(Eigen::Matrix2d::Identity());
            auto rk = new g2o::RobustKernelHuber();
            rk->setDelta(chi2_th);
            edge->setRobustKernel(rk);
            edges_and_features.insert({edge, feat});

            optimizer.addEdge(edge);
            index++;
        }
    }

    // do optimization and eliminate the outliers
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    int count_outlier = 0, count_inlier = 0;
    int iteration = 0;
    while (iteration < 5) {
        count_outlier = 0;
        count_inlier = 0;
        // determine if we want to adjust the outlier threshold
        for (auto &ef : edges_and_features) {
            if (ef.first->chi2() > chi2_th) {
                count_outlier++;
            } else {
                count_inlier++;
            }
        }
        double inlier_ratio = count_inlier / double(count_inlier + count_outlier);
        if (inlier_ratio > 0.5) {
            break;
        } else {
            chi2_th *= 2;
            iteration++;
        }
    }

    for (auto &ef : edges_and_features) {
        if (ef.first->chi2() > chi2_th) {
            ef.second->is_outlier = true;
            // remove the observation
            ef.second->map_point_.lock()->removeObservation(ef.second);
        } else {
            ef.second->is_outlier = false;
        }
    }

    cout << "Outlier/Inlier in optimization: " << count_outlier << "/" << count_inlier << endl;

    // Set pose and lanrmark position
    for (auto &v : vertices) {
        keyframes.at(v.first)->SetPose(v.second->estimate());
    }
    for (auto &v : vertices_landmarks) {
        landmarks.at(v.first)->setPos(v.second->estimate());
    }
}