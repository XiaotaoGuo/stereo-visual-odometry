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
    Map::WkPtr map_;
    std::string window_name_;
    bool& end_signal_;

public:
    MapVisualizer(const Map::WkPtr map, bool& end_signal)
        : map_(map),
          window_name_("Map and Trajectroy"),
          end_signal_(end_signal) {
        SetUp();
    }

    void Foward() const {
        // fetch the context and bind it to this thread
        pangolin::BindToContext(window_name_);

        // we manually need to restore the properties of the context
        glEnable(GL_DEPTH_TEST);

        // Define Projection and initial ModelView matrix
        pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100),
            pangolin::ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin::AxisY));

        // Create Interactive View in window
        pangolin::Handler3D handler(s_cam);
        pangolin::View& d_cam =
            pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f)
                .SetHandler(&handler);

        while (!pangolin::ShouldQuit() && !end_signal_) {
            // Clear screen and activate view to render into
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(s_cam);

            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

            // Render OpenGL Cube
            // pangolin::glDrawColouredCube();
            DrawMapPoints();

            // Swap frames and Process Events
            pangolin::FinishFrame();
        }

        // unset the current context from the main thread
        pangolin::GetBoundWindow()->RemoveCurrent();
    }

private:
    void SetUp() {
        // create a window and bind its context to the main thread
        pangolin::CreateWindowAndBind(window_name_, 640, 480);

        // enable depth
        glEnable(GL_DEPTH_TEST);

        // unset the current context from the main thread
        pangolin::GetBoundWindow()->RemoveCurrent();
    }

    void DrawMapPoints() const {
        glPointSize(10);
        glBegin(GL_POINTS);

        for (const auto& point : map_.lock()->GetAllMapPoints()) {
            glColor3f(0.0, 0.0, 1.0);
            Eigen::Vector3d pos = point.second->Pos();
            glVertex3d(pos.x(), pos.y(), pos.z());
            std::cout << "Added 1 point" << std::endl;
        }
        glEnd();
    }
};
}  // namespace stereo_visual_odometry

#endif