#pragma once
#include "STrack.h"
#include "KalmanFilter.h"
#include "lapjv.h"



class ByteTrack{
public:
    ByteTrack(int max_time_lost = 30, float detect_thresh = 0.5, float high_thresh = 0.6, float match_thresh_1 = 0.8, float match_thresh_2 = 0.5);
    ~ByteTrack();
    std::vector<STrack> init(std::vector<Object3D> objects);
    std::vector<STrack> update(std::vector<Object3D> objects);

private:
    float detect_thresh;
    float high_thresh;
    float match_thresh_1;
    float match_thresh_2;
    int frame_id;
    int max_time_lost;

    vector<STrack> stracks;
};
