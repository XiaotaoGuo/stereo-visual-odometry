#include "viewer.h"
#include "Feature.h"
#include "Frame.h"

#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

Viewer::Viewer() { viewer_thread_ = thread(bind(&Viewer::ThreadLoop, this)); }

void Viewer::Close() {
    viewer_running_ = false;
    viewer_thread_.join();
}

void Viewer::AddCurrentFrame(FramePtr current_frame) {
    unique_lock<mutex> lck(viewer_data_mutex_);
    last_frame_ = current_frame_;
    current_frame_ = current_frame;
}

void Viewer::UpdateMap() {
    unique_lock<mutex> lck(viewer_data_mutex_);
    assert(map_ != nullptr);
    // active_keyframes_ = map_->getAllFrames();
    active_keyframes_ = map_->getActiveKeyFrames();
    active_landmarks_ = map_->getActiveMapPoints();
    allFrames_ = map_->getAllFrames();
    map_updated_ = true;
}

void Viewer::ThreadLoop() {
    pangolin::CreateWindowAndBind("SLAM", (1024 + 620), 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState vis_camera(
        pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& vis_display =
        pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(620), 1.0,
                       -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(vis_camera));

    pangolin::View& traj_img =
        pangolin::Display("Trajectory")
            .SetBounds(pangolin::Attach::Pix(2 * height_), 1.0f, 0.0f,
                       pangolin::Attach::Pix(width_), -1.0 * width_ / 392.0)
            .SetLock(pangolin::LockLeft, pangolin::LockTop);

    pangolin::View& left_img =
        pangolin::Display("Left")
            .SetBounds(pangolin::Attach::Pix(0), pangolin::Attach::Pix(height_),
                       pangolin::Attach::Pix(0), pangolin::Attach::Pix(width_),
                       -1.0 * width_ / height_)
            .SetLock(pangolin::LockLeft, pangolin::LockBottom);

    pangolin::View& right_img =
        pangolin::Display("Right")
            .SetBounds(pangolin::Attach::Pix(height_),
                       pangolin::Attach::Pix(height_ * 2),
                       pangolin::Attach::Pix(0), pangolin::Attach::Pix(width_),
                       -1.0 * width_ / height_)
            .SetLock(pangolin::LockLeft, pangolin::LockBottom);

    pangolin::GlTexture traj_texture_ = pangolin::GlTexture(
        width_, 392.0, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);

    pangolin::GlTexture left_texture_ = pangolin::GlTexture(
        width_, height_, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);
    pangolin::GlTexture right_texture_ = pangolin::GlTexture(
        width_, height_, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);

    const float blue[3] = {0, 0, 1};
    const float green[3] = {0, 1, 0};

    top_down_tajectory =
        cv::Mat(768 - 2 * height_, width_, CV_8UC3, cv::Scalar(0, 0, 0));

    while (!pangolin::ShouldQuit() && viewer_running_) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        vis_display.Activate(vis_camera);

        unique_lock<mutex> lock(viewer_data_mutex_);

        if (map_) {
            DrawMapPoints();
        }
        cv::Mat last_frame_img(height_, width_, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Mat current_frame_img(height_, width_, CV_8UC3,
                                  cv::Scalar(0, 0, 0));

        if (last_frame_) {
            last_frame_img = PlotFrameImage(last_frame_);
        } else {
            cv::putText(last_frame_img, "No last frame",
                        cv::Point2d(width_ / 2, height_ / 2),
                        cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 0, 0));
        }

        if (current_frame_) {
            DrawFrame(current_frame_, green);
            FollowCurrentFrame(vis_camera);

            current_frame_img = PlotFrameImage(current_frame_);

        } else {
            cv::putText(current_frame_img, "No current frame",
                        cv::Point2d(width_ / 2, height_ / 2),
                        cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 0, 0));
        }

        glColor3f(1, 1, 1);
        left_texture_.Upload(current_frame_img.data, GL_BGR, GL_UNSIGNED_BYTE);
        left_img.Activate();
        left_texture_.RenderToViewportFlipY();

        right_texture_.Upload(last_frame_img.data, GL_BGR, GL_UNSIGNED_BYTE);
        right_img.Activate();
        right_texture_.RenderToViewportFlipY();

        glColor3f(1.0f, 1.0f, 1.0f);
        UpdateTopDownTrajectory();
        traj_texture_.Upload(top_down_tajectory.data, GL_BGR, GL_UNSIGNED_BYTE);
        traj_img.Activate();
        traj_texture_.RenderToViewportFlipY();

        pangolin::FinishFrame();
        usleep(5000);
    }

    std::cout << "Stop viewer" << std::endl;
}

cv::Mat Viewer::PlotFrameImage(const FramePtr frame) {
    cv::Mat img_out;
    cv::cvtColor(frame->left_img_, img_out, CV_GRAY2BGR);
    for (size_t i = 0; i < frame->left_features_.size(); ++i) {
        if (frame->left_features_[i]->map_point_.lock()) {
            auto feat = frame->left_features_[i];
            cv::circle(img_out, feat->position_.pt, 1, cv::Scalar(0, 250, 0),
                       1);
        }
    }
    return img_out;
}

void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera) {
    Sophus::SE3d Twc = current_frame_->Pose().inverse();
    pangolin::OpenGlMatrix m(Twc.matrix());
    vis_camera.Follow(m, true);
}

void Viewer::UpdateTopDownTrajectory() {
    top_down_tajectory.setTo(cv::Scalar(0, 0, 0));

    static double resolution = 1;
    static double top_left_x = -0.5 * top_down_tajectory.cols * resolution;
    static double top_left_y = -0.5 * top_down_tajectory.rows * resolution;

    cv::putText(top_down_tajectory,             // target image
                "Trajectory in top-down view",  // text
                cv::Point(10,
                          20),  // top-left position
                cv::FONT_HERSHEY_DUPLEX, 1.0,
                CV_RGB(118, 185, 0),  // font color
                2);

    for (auto frame : allFrames_) {
        double point_x =
            (frame.second->Pose().translation().x() - top_left_x) / resolution;
        double point_y =
            (frame.second->Pose().translation().z() - top_left_y) / resolution;
        cv::Point2d loc(point_x, point_y);
        cv::circle(top_down_tajectory, loc, 0, cv::Scalar(0, 255, 210), 2);
    }
}

void Viewer::DrawFrame(FramePtr frame, const float* color) {
    Sophus::SE3d Twc = frame->Pose().inverse();
    const float sz = 1.0;
    const int line_width = 2.0;
    const float fx = 400;
    const float fy = 400;
    const float cx = 512;
    const float cy = 384;
    const float width = 1080;
    const float height = 768;

    glPushMatrix();

    Sophus::Matrix4f m = Twc.matrix().template cast<float>();
    glMultMatrixf((GLfloat*)m.data());

    if (color == nullptr) {
        glColor3f(1, 0, 0);
    } else {
        glColor3f(color[0], color[1], color[2]);
    }

    glLineWidth(line_width);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glEnd();
    glPopMatrix();
}

void Viewer::DrawMapPoints() {
    const float red[3] = {1.0, 0, 0};
    for (auto& kf : active_keyframes_) {
        DrawFrame(kf.second, red);
    }

    glPointSize(2);
    glBegin(GL_POINTS);
    for (auto& landmark : active_landmarks_) {
        auto pos = landmark.second->Pos();
        glColor3f(red[0], red[1], red[2]);
        glVertex3d(pos[0], pos[1], pos[2]);
    }
    glEnd();
}
