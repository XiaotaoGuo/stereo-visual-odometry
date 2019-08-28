//
// Created by guoxt on 19-8-24.
//

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
    typedef shared_ptr<MapPoint> Ptr;

    unsigned long id_;
    bool is_outlier_;
    Eigen::Vector3d pos_;
    mutex data_mutex_;
    int observed_times_;
    list<weak_ptr<Feature>> observations_;

    MapPoint();
    MapPoint(long id, Eigen::Vector3d position);

    Eigen::Vector3d Pos();
    void setPos(const Eigen::Vector3d &pos);

    void AddObservation(shared_ptr<Feature> feature);
    void RemoveObservation(shared_ptr<Feature> feature);

    list<weak_ptr<Feature>> getObservations();

    static MapPoint::Ptr CreateNewMappoint();
};


#endif //STEORO_VISUAL_ODOMETRY_MAPPOINT_H
