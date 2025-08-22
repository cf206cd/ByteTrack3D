#pragma once

#include <opencv2/opencv.hpp>
#include "KalmanFilter.h"
#include "object.h"

using namespace cv;
using namespace std;

enum TrackState { New = 0, Tracked, Lost, Removed };

class STrack
{
public:
	STrack(Object3D object3d_);
	void mark_lost();
	void mark_removed();
	int next_id();
	void update(Object3D object3d_);
	void predict();

public:
	Object3D object3d;
	int track_id;
	TrackState state;
	int lost_frame;

private:
	EKF_CTRA kalman_filter_pose;
	KF_SIZE kalman_filter_size;
	KF_YAW kalman_filter_yaw;
};

