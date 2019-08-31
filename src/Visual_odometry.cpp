
#include "Visual_odometry.h"
#include <boost/format.hpp>
#include <opencv2/opencv.hpp>

Visual_odometry::Visual_odometry() {}

bool Visual_odometry::init() {
    frontend_ = shared_ptr<FrontEnd>(new FrontEnd);
    backend_ = shared_ptr<BackEnd>(new BackEnd);
    map_ = shared_ptr<Map>(new Map);
    viewer_ = shared_ptr<Viewer>(new Viewer);
    init_camera();
    frontend_->setBackend(backend_);
    frontend_->setViewer(viewer_);
    frontend_->setMap(map_);
    frontend_->setCameras(cam1_, cam2_);
    viewer_->SetMap(map_);

    backend_->setMap(map_);
    backend_->setCameras(cam1_, cam2_);

    current_index = 0;
    return true;


}

bool Visual_odometry::init_camera() {
    double fx1, fy1, cx1, cy1;
    fx1 = 7.188560000000e+02;
    fy1 = 7.188560000000e+02;
    cx1 = 6.071928000000e+02;
    cy1 = 1.852157000000e+02;
    Eigen::Matrix3d K1;
    Eigen::Vector3d t1(0.0, 0.0, 0.0);
    K1 << fx1, 0.0, cx1,
         0.0, fy1, cy1,
         0.0, 0.0, 1;
    t1 = K1.inverse() * t1;
    K1 = K1 * 0.5;
    cam1_ = shared_ptr<Camera>(new Camera(K1(0,0), K1(1,1), K1(0,2), K1(1,2), t1.norm(), Sophus::SE3d(Sophus::SO3d(), t1 )));

    double fx2, fy2, cx2, cy2;
    fx2 = 7.188560000000e+02;
    fy2 = 7.188560000000e+02;
    cx2 = 6.071928000000e+02;
    cy2 = 1.852157000000e+02;
    Eigen::Matrix3d K2;
    Eigen::Vector3d t2(-3.861448000000e+02, 0.0, 0.0);
    K2 << fx2, 0.0, cx2,
            0.0, fy2, cy2,
            0.0, 0.0, 1;
    t2 = K2.inverse() * t2;
    K2 = K2 * 0.5;
    cam2_ = shared_ptr<Camera>(new Camera(K2(0,0), K2(1,1), K2(0,2), K2(1,2), t2.norm(), Sophus::SE3d(Sophus::SO3d(), t2 )));

}

void Visual_odometry::start() {
    while(true){
        cout << "VO is running" << endl;
        if(forward() == false){
            break;
        }
    }

    backend_->stop();
    viewer_->Close();

    cout << "VO closed" << endl;
}

bool Visual_odometry::forward() {

    cv::Mat left_image, right_image;
    boost::format fmt("%s/image_%d/%06d.png");
    string dataset_path = "/home/guoxt/Downloads/data_odometry_gray/dataset/sequences/00/";
    left_image = cv::imread((fmt % dataset_path % 0 % current_index).str(), cv::IMREAD_GRAYSCALE);
    right_image = cv::imread((fmt % dataset_path % 1 % current_index).str(), cv::IMREAD_GRAYSCALE);
    if(!left_image.cols) return false;
    cv::Mat image_left_resized, image_right_resized;
    cv::resize(left_image, image_left_resized, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);
    cv::resize(right_image, image_right_resized, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);

    auto new_frame = Frame::CreateFrame();
    new_frame->left_img_ = image_left_resized;
    new_frame->right_img_ = image_right_resized;
    current_index++;

    bool success = frontend_->addFrame(new_frame);

    return success;
}
