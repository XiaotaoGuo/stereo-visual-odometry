#ifndef STEORO_VISUAL_ODOMETRY_MAPPOINT_H
#define STEORO_VISUAL_ODOMETRY_MAPPOINT_H

#include "config.h"
#include "Frame.h"
#include "Feature.h"

class Feature;
class Frame;

class MapPoint {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    unsigned long id_ = 0;
    bool is_outlier_ = false;
    Eigen::Vector3d pos_ = Eigen::Vector3d::Zero();
    mutex data_mutex_;
    int observed_times_ = 0;
    list<weak_ptr<Feature>> observations_;

    MapPoint();
    MapPoint(long id, Eigen::Vector3d position);

    Eigen::Vector3d Pos();
    void setPos(const Eigen::Vector3d &pos);

    void addObservation(shared_ptr<Feature> feature);
    void removeObservation(shared_ptr<Feature> feature);

    list<weak_ptr<Feature>> getObservations();

    static MapPointPtr createNewMappoint();
};


#endif //STEORO_VISUAL_ODOMETRY_MAPPOINT_H
