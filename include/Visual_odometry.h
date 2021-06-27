#ifndef STEORO_VISUAL_ODOMETRY_VISUAL_ODOMETRY_H
#define STEORO_VISUAL_ODOMETRY_VISUAL_ODOMETRY_H

#include "config.h"
#include "dataset/data_reader.h"

class Map;
class FrontEnd;
class BackEnd;
class Viewer;

class Visual_odometry {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Visual_odometry();

    bool init_camera();
    bool init(string data_path, int32_t seq_id);
    void start();
    bool forward();
    bool stop();

private:
    std::shared_ptr<Camera> cam1_;
    std::shared_ptr<Camera> cam2_;
    std::shared_ptr<Map> map_;
    std::shared_ptr<FrontEnd> frontend_;
    std::shared_ptr<BackEnd> backend_;
    std::shared_ptr<Viewer> viewer_;
    DatasetReader::Ptr dataset_reader_;
    int current_index;

    string dataset_root_path_;
    string seq_id_;

    double average_times_;

    bool inited_;
};

#endif  // STEORO_VISUAL_ODOMETRY_VISUAL_ODOMETRY_H
