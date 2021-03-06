
#include "Visual_odometry.h"
#include <boost/format.hpp>
#include <opencv2/opencv.hpp>

Visual_odometry::Visual_odometry() {}

bool Visual_odometry::init(string data_path, string seq_id) {
    dataset_root_path_ = data_path;
    seq_id_ = seq_id;

    frontend_ = shared_ptr<FrontEnd>(new FrontEnd);
    backend_ = shared_ptr<BackEnd>(new BackEnd);
    map_ = shared_ptr<Map>(new Map);
    viewer_ = shared_ptr<Viewer>(new Viewer);
    init_camera();
    frontend_->setBackend(backend_);
    frontend_->setViewer(viewer_);
    frontend_->setMap(map_);
    frontend_->setCameras(cam1_, cam2_);
    viewer_->setMap(map_);

    backend_->setMap(map_);
    backend_->setCameras(cam1_, cam2_);

    current_index = 0;
    average_times_ = 0.0;
    return true;


}

bool Visual_odometry::init_camera() {

    ifstream fin(dataset_root_path_ + seq_id_ + "/calib.txt");
    if (!fin) {
        cout << "cannot find " << dataset_root_path_ << seq_id_ << "/calib.txt!" << endl;
        return false;
    }

    int idx = 0;
    while(idx < 2) {
        char camera_name[3];
        for (int k = 0; k < 3; ++k) {
            fin >> camera_name[k];
        }
        double projection_data[12];
        for (int k = 0; k < 12; ++k) {
            fin >> projection_data[k];
        }

        Eigen::Matrix3d K;
        Eigen::Vector3d t(0.0, 0.0, 0.0);
        K << projection_data[0], projection_data[1], projection_data[2],
            projection_data[4], projection_data[5], projection_data[6],
            projection_data[8], projection_data[9], projection_data[10];
        t << projection_data[3], projection_data[7], projection_data[11];
        t = K.inverse() * t;
        K = K * 0.5;
        if(idx == 0)
            cam1_ = shared_ptr<Camera>(new Camera(K(0,0), K(1,1), K(0,2), K(1,2), t.norm(), Sophus::SE3d(Sophus::SO3d(), t )));
        if(idx == 1)
            cam2_ = shared_ptr<Camera>(new Camera(K(0,0), K(1,1), K(0,2), K(1,2), t.norm(), Sophus::SE3d(Sophus::SO3d(), t )));
        idx++;
    }

}

void Visual_odometry::start() {
    while(true){
        cout << "VO is running" << endl;
        if(forward() == false){
            break;
        }
    }
    FrameType allFrames_ = map_->getAllFrames();
    ofstream outfile;
    outfile.open(dataset_root_path_ + seq_id_ + "/result.txt");
    for(int i = 0; i < allFrames_.size(); i++){
        if(allFrames_[i]->is_keyframe_){
            outfile << "key: ";
        }
        else{
            outfile << "other: ";
        }
        auto pose_ = allFrames_[i]->Pose();
        Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic, Eigen::RowMajor> pose_vec = pose_.matrix3x4();
        pose_vec.resize(1,12);
        outfile << pose_vec << "\n";

    }
    outfile.close();
    backend_->stop();
    viewer_->Close();

    cout << "VO closed" << endl;
}

bool Visual_odometry::forward() {

    cv::Mat left_image, right_image;
    boost::format fmt("%s/image_%d/%06d.png");
    left_image = cv::imread((fmt % (dataset_root_path_ + seq_id_) % 0 % current_index).str(), cv::IMREAD_GRAYSCALE);
    right_image = cv::imread((fmt % (dataset_root_path_ + seq_id_) % 1 % current_index).str(), cv::IMREAD_GRAYSCALE);
    if(!left_image.cols) return false;
    cv::Mat image_left_resized, image_right_resized;
    cv::resize(left_image, image_left_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
    cv::resize(right_image, image_right_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);

    auto new_frame = Frame::CreateFrame();
    new_frame->left_img_ = image_left_resized;
    new_frame->right_img_ = image_right_resized;
    current_index++;
    auto t1 = chrono::steady_clock::now();
    bool success = frontend_->addFrame(new_frame);
    auto t2 = chrono::steady_clock::now();
    auto time_used = chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    average_times_ = (average_times_ * (current_index-1) + time_used.count()) / double(current_index);
    cout << "VO cost time averaged: " << average_times_ << " seconds for GFTT." << endl;

    return success;
}
