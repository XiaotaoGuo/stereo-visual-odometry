#include "Map.h"
#include "Frame.h"
#include "MapPoint.h"

Map::Map() {}

void Map::insertFrame(FramePtr frame){
    if(frames_.find(frame->id_) == frames_.end()){
        frames_.insert(make_pair(frame->id_, frame));
    }
    else{ //basically won't be used
        keyframes_[frame->id_] = frame;
    }
}

void Map::insertKeyFrame(FramePtr frame) {
    current_frame_ = frame;
    if(keyframes_.find(frame->keyframe_id_) == keyframes_.end()){
        keyframes_.insert(make_pair(frame->keyframe_id_, frame));
        active_keyframes_.insert(make_pair(frame->keyframe_id_, frame));
    }
    else{
        keyframes_[frame->keyframe_id_] = frame;
        active_keyframes_[frame->keyframe_id_] = frame;
    }

    if(active_keyframes_.size() > num_active_keyframes_){
        removeOldKeyframe();
    }
}

void Map::insertMapPoint(shared_ptr<MapPoint> mappoint) {
    if(landmarks_.find(mappoint->id_) == landmarks_.end()){
        landmarks_.insert(make_pair(mappoint->id_, mappoint));
        active_landmarks_.insert(make_pair(mappoint->id_, mappoint));
    }
    else{
        landmarks_[mappoint->id_] = mappoint;
        active_landmarks_[mappoint->id_] = mappoint;
    }

}

LandmarkType Map::getAllMapPoints() {
    unique_lock<mutex> lck(data_mutex);
    return landmarks_;
}

FrameType Map::getAllKeyframes() {
    unique_lock<mutex> lck(data_mutex);
    return keyframes_;
}

LandmarkType Map::getActiveMapPoints(){
    unique_lock<std::mutex> lck(data_mutex);
    return active_landmarks_;
}

FrameType Map::getActiveKeyFrames() {
    unique_lock<mutex> lck(data_mutex);
    return active_keyframes_;
}

FrameType Map::getAllFrames() {
    unique_lock<mutex> lck(data_mutex);
    return frames_;
}

void Map::removeOldKeyframe() {
    if (current_frame_ == nullptr) return;
    // find nearest (or furthest if can't find the near frame) frame to delete
    double max_dis = 0, min_dis = INT_MAX;
    double max_kf_id = 0, min_kf_id = 0;
    auto Twc = current_frame_->Pose().inverse(); // current -> world
    for (auto& kf : active_keyframes_) {
        if (kf.second == current_frame_) continue;
        auto dis = (kf.second->Pose() * Twc).log().norm(); //calculate distance
        if (dis > max_dis) {
            max_dis = dis;
            max_kf_id = kf.first;
        }
        if (dis < min_dis) {
            min_dis = dis;
            min_kf_id = kf.first;
        }
    }

    const double min_dis_th = 0.2;  // threshold for nearest frame
    FramePtr frame_to_remove = nullptr;
    if (min_dis < min_dis_th) {
        // perfer delete nearest frame
        frame_to_remove = keyframes_.at(min_kf_id);
    } else {
        // delete furthest frame
        frame_to_remove = keyframes_.at(max_kf_id);
    }

    //cout << "remove keyframe: " << frame_to_remove->keyframe_id_ << endl;
    // remove keyframe and landmark observation
    active_keyframes_.erase(frame_to_remove->keyframe_id_);
    for (auto feat : frame_to_remove->left_features_) {
        auto mp = feat->map_point_.lock();
        if (mp) {
            mp->removeObservation(feat);
        }
    }
    for (auto feat : frame_to_remove->right_features_) {
        if (feat == nullptr) continue;
        auto mp = feat->map_point_.lock();
        if (mp) {
            mp->removeObservation(feat);
        }
    }

    cleanMap();
}

void Map::cleanMap() {
    int cnt_landmark_removed = 0;
    for (auto iter = active_landmarks_.begin();iter != active_landmarks_.end();) {
        if (iter->second->observed_times_ == 0) {
            iter = active_landmarks_.erase(iter);
            cnt_landmark_removed++;
        } else {
            ++iter;
        }
    }
    //cout << "Removed " << cnt_landmark_removed << " active landmarks" << endl;
}