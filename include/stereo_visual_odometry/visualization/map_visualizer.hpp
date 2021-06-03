/**
 * Ref: https://blog.csdn.net/weixin_43991178/article/details/105174734
 */

#ifndef VISUALIZATION_MAP_VISUALIZER_H_
#define VISUALIZATION_MAP_VISUALIZER_H_

#include <memory>

#include <pangolin/pangolin.h>

#include "stereo_visual_odometry/types/map.hpp"

namespace stereo_visual_odometry {

class MapVisualizer {
public:
    using Ptr = std::shared_ptr<MapVisualizer>;

private:
    Map::Ptr map_;
    std::string window_name_;
    bool& end_signal_;
    pangolin::OpenGlRenderState s_cam_;
    pangolin::View d_cam_, traj_img_, left_img_, right_img_;
    pangolin::GlTexture traj_texture_, left_texture_, right_texture_;
    int height_;
    int width_;

public:
    MapVisualizer(const Map::Ptr map, bool& end_signal)
        : map_(map),
          window_name_("Map and Trajectroy"),
          end_signal_(end_signal) {
        SetUp();
    }

    void Foward() {
        // fetch the context and bind it to this thread
        pangolin::BindToContext(window_name_);

        // we manually need to restore the properties of the context
        glEnable(GL_DEPTH_TEST);

        width_ = 1241 / 2;
        height_ = 376 / 2;

        // Define Projection and initial ModelView matrix
        s_cam_ = pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(940, 940, 420, 420, 470, 470, 0.2, 100),
            pangolin::ModelViewLookAt(-1, 0, 1, 0, 0, 0, pangolin::AxisX));

        // Create Interactive View in window
        pangolin::Handler3D handler(s_cam_);
        d_cam_ = pangolin::CreateDisplay()
                     .SetBounds(0.0, 1.0, pangolin::Attach::Pix(620), 1.0,
                                -940.0f / 940.0f)
                     .SetHandler(&handler);

        // create view for showing stereo images with features

        traj_img_ = pangolin::Display("Trajectory")
                        .SetBounds(pangolin::Attach::Pix(2 * height_), 1.0f,
                                   0.0f, pangolin::Attach::Pix(width_),
                                   -1.0 * width_ / (height_ * 3.0))
                        .SetLock(pangolin::LockLeft, pangolin::LockTop);

        left_img_ = pangolin::Display("Left")
                        .SetBounds(pangolin::Attach::Pix(0),
                                   pangolin::Attach::Pix(height_),
                                   pangolin::Attach::Pix(0),
                                   pangolin::Attach::Pix(width_),
                                   -1.0 * width_ / height_)
                        .SetLock(pangolin::LockLeft, pangolin::LockBottom);

        right_img_ = pangolin::Display("Right")
                         .SetBounds(pangolin::Attach::Pix(height_),
                                    pangolin::Attach::Pix(height_ * 2),
                                    pangolin::Attach::Pix(0),
                                    pangolin::Attach::Pix(width_),
                                    -1.0 * width_ / height_)
                         .SetLock(pangolin::LockLeft, pangolin::LockBottom);

        traj_texture_ = pangolin::GlTexture(width_, 3 * height_, GL_RGB, false,
                                            0, GL_BGR, GL_UNSIGNED_BYTE);

        left_texture_ = pangolin::GlTexture(width_, height_, GL_RGB, false, 0,
                                            GL_BGR, GL_UNSIGNED_BYTE);
        right_texture_ = pangolin::GlTexture(width_, height_, GL_RGB, false, 0,
                                             GL_BGR, GL_UNSIGNED_BYTE);

        while (!pangolin::ShouldQuit() && !end_signal_) {
            // Clear screen and activate view to render into
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam_.Activate(s_cam_);

            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

            // Render OpenGL Cube
            // pangolin::glDrawColouredCube();
            map_->Lock();
            auto all_frames = map_->GetAllFrames();
            for (auto frame : all_frames) {
                if (frame.second == map_->GetLatestFrame()) {
                    DrawEgo(frame.second->Pose(), 1, 0, 0);
                } else {
                    DrawEgo(frame.second->Pose(), 0, 1, 0);
                }
            }
            DrawMapPoints();

            // Swap frames and Process Events
            if (map_->GetAllFrames().size() != 0) {
                DrawTrajectory();
                DrawFrame();
            }

            map_->Unlock();
            pangolin::FinishFrame();
        }

        // unset the current context from the main thread
        pangolin::GetBoundWindow()->RemoveCurrent();
    }

private:
    void SetUp() {
        // create a window and bind its context to the main thread
        pangolin::CreateWindowAndBind(window_name_, 1560, 940);

        // enable depth
        glEnable(GL_DEPTH_TEST);

        // unset the current context from the main thread
        pangolin::GetBoundWindow()->RemoveCurrent();
    }

    void DrawMapPoints() const {
        glPointSize(10);
        glBegin(GL_POINTS);

        for (const auto& point : map_->GetAllMapPoints()) {
            glColor3f(0.0, 0.0, 1.0);
            Eigen::Vector3d pos = point.second->Pos();
            glVertex3d(pos.x(), pos.y(), pos.z());
        }
        glEnd();
    }

    void DrawEgo(const Sophus::SE3d& pose, float r, float g, float b) const {
        constexpr float w = 0.2;
        constexpr float h = w * 0.75;
        constexpr float d = w * 0.6;

        glPushMatrix();
        Eigen::Matrix4d t_o_w;
        t_o_w << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1;
        Sophus::Matrix4d transformation = pose.matrix();
        glMultMatrixd((GLdouble*)transformation.data());

        glLineWidth(2);
        glBegin(GL_LINES);
        glColor3f(r, g, b);
        glVertex3f(0, 0, 0);
        glVertex3f(d, w, h);

        glVertex3f(0, 0, 0);
        glVertex3f(d, -w, h);

        glVertex3f(0, 0, 0);
        glVertex3f(d, -w, -h);

        glVertex3f(0, 0, 0);
        glVertex3f(d, w, -h);

        glVertex3f(d, w, h);
        glVertex3f(d, -w, h);

        glVertex3f(d, -w, h);
        glVertex3f(d, -w, -h);

        glVertex3f(d, -w, -h);
        glVertex3f(d, w, -h);

        glVertex3f(d, w, -h);
        glVertex3f(d, w, h);
        glEnd();
        glPopMatrix();
    }

    void DrawTrajectory() {
        glColor3f(1.0f, 1.0f, 1.0f);

        cv::Mat traj(3 * height_, width_, CV_8UC3, cv::Scalar(0));
        cv::putText(traj,                           // target image
                    "Trajectory in top-down view",  // text
                    cv::Point(10,
                              20),  // top-left position
                    cv::FONT_HERSHEY_DUPLEX, 1.0,
                    CV_RGB(118, 185, 0),  // font color
                    2);
        auto all_frames = map_->GetAllFrames();
        for (auto frame : all_frames) {
            cv::Point2d loc(
                width_ * 0.5 + (frame.second->Pose().translation().x() * 10.0),
                1.5 * height_ +
                    (frame.second->Pose().translation().y()) * 10.0);
            cv::circle(traj, loc, 0, cv::Scalar(0, 255, 210), 10);
        }
        traj_texture_.Upload(traj.data, GL_BGR, GL_UNSIGNED_BYTE);
        traj_img_.Activate();
        traj_texture_.RenderToViewportFlipY();
    }

    void DrawFrame() {
        cv::Mat left_bgr = map_->GetLatestFrame()->Images().first;
        cv::Mat right_bgr = map_->GetLatestFrame()->Images().second;

        left_texture_.Upload(left_bgr.data, GL_BGR, GL_UNSIGNED_BYTE);
        left_img_.Activate();
        left_texture_.RenderToViewportFlipY();

        right_texture_.Upload(right_bgr.data, GL_BGR, GL_UNSIGNED_BYTE);
        right_img_.Activate();
        right_texture_.RenderToViewportFlipY();
    }
};
}  // namespace stereo_visual_odometry

#endif