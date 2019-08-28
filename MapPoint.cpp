//
// Created by guoxt on 19-8-24.
//

#include "MapPoint.h"
#include "Feature.h"

class Feature;
class Frame;

MapPoint::MapPoint() {}

MapPoint::MapPoint(long id, Eigen::Vector3d position):id_(id), pos_(position) {}

Eigen::Vector3d MapPoint::Pos() {
    unique_lock<mutex> lck(data_mutex_);
    return pos_;
}

void MapPoint::setPos(const Eigen::Vector3d &pos) {
    unique_lock<mutex> lck(data_mutex_);
    pos_ = pos;
}

MapPoint::Ptr MapPoint::CreateNewMappoint() {
    static long factory_id = 0;
    MapPoint::Ptr new_mappoint(new MapPoint);
    new_mappoint->id_ = factory_id++;
    return new_mappoint;
}

void MapPoint::AddObservation(Feature::Ptr feature) {
    unique_lock<mutex> lck(data_mutex_);
    observations_.push_back(feature);
    observed_times_++;
}

void MapPoint::RemoveObservation(Feature::Ptr feature) {
    unique_lock<mutex> lck(data_mutex_);
    for(auto iter = observations_.begin(); iter != observations_.end(); iter++){
        if(iter->lock() == feature){
            observations_.erase(iter);
            feature->map_point_.reset();
            observed_times_--;
            break;
        }
    }
}

list<weak_ptr<Feature>> MapPoint::getObservations() {
    unique_lock<mutex> lck(data_mutex_);
    return observations_;
}
