#include "STrack.h"
#include <map>
#include <algorithm>

STrack::STrack(Object3D object3d_)
	: 
	track_id(next_id()),
	state(TrackState::New),
	lost_frame(0),
	kalman_filter_pose(0.1),
	kalman_filter_size(0.1),
	kalman_filter_yaw(0.1)
{
	object3d = object3d_;
	// 初始化位姿相关的协方差矩阵
	Eigen::VectorXd diag_P_pose(6);
	diag_P_pose << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
	Eigen::MatrixXd P_pose = diag_P_pose.asDiagonal();

	Eigen::VectorXd diag_Q_pose(6);
	diag_Q_pose << 2.0, 2.0, 1.0, 0.5, 1.0, 1.5;
	Eigen::MatrixXd Q_pose = diag_Q_pose.asDiagonal();

	Eigen::VectorXd diag_R_pose(2);
	diag_R_pose << 0.5, 0.5;
	Eigen::MatrixXd R_pose = diag_R_pose.asDiagonal();

	Eigen::VectorXd init_x_pose(6);
	init_x_pose << object3d.x, object3d.y, 0.1, 0.1, 0.1, 0.1;
	kalman_filter_pose = EKF_CTRA(0.1, P_pose, Q_pose, R_pose, init_x_pose);

	Eigen::VectorXd diag_P_size(4);
	diag_P_size << 1.0, 1.0, 10.0, 10.0;
	Eigen::MatrixXd P_size = diag_P_size.asDiagonal();

	Eigen::VectorXd diag_Q_size(4);
	diag_Q_size << 0.5, 0.5, 5, 5;
	Eigen::MatrixXd Q_size = diag_Q_size.asDiagonal();

	Eigen::VectorXd diag_R_size(2);
	diag_R_size << 0.02, 0.02;
	Eigen::MatrixXd R_size = diag_R_size.asDiagonal();

	Eigen::VectorXd init_x_size(4);
	init_x_size << object3d.l, object3d.w, 0, 0;
	kalman_filter_size = KF_SIZE(0.1, P_size, Q_size, R_size, init_x_size);

	Eigen::VectorXd diag_P_yaw(2);
	diag_P_yaw << 0.1, 0.1;
	Eigen::MatrixXd P_yaw = diag_P_yaw.asDiagonal();

	Eigen::VectorXd diag_Q_yaw(2);
	diag_Q_yaw << 0.1, 0.1;
	Eigen::MatrixXd Q_yaw = diag_Q_yaw.asDiagonal();

	Eigen::VectorXd diag_R_yaw(2);
	diag_R_yaw << 0.2, 5.0;
	Eigen::MatrixXd R_yaw = diag_R_yaw.asDiagonal();
	Eigen::VectorXd init_x_yaw(2);
	object3d.yaw = norm_radian(object3d.yaw);
	float vel_yaw = norm_radian(atan2(object3d.vy,object3d.vx));
	init_x_yaw << object3d.yaw, vel_yaw;
	kalman_filter_yaw = KF_YAW(0.1, P_yaw, Q_yaw, R_yaw, init_x_yaw);
}

void STrack::update(Object3D object3d_)
{
	
	object3d = object3d_;
	std::cout << "更新 object3d: x = " << object3d.x 
              << ", y = " << object3d.y 
              << ", l = " << object3d.l 
              << ", w = " << object3d.w 
              << ", yaw = " << object3d.yaw 
              << ", vx = " << object3d.vx 
              << ", vy = " << object3d.vy << std::endl;
	// 在进行卡尔曼滤波更新前，对 object3d.yaw 进行归一化处理
	object3d.yaw = norm_radian(object3d.yaw);
	std::cout << "归一化后 object3d.yaw = " << object3d.yaw << std::endl;

	// 显式指定向量大小，避免潜在的未初始化问题
	Eigen::VectorXd pose_measurement(2);
	pose_measurement << object3d.x, object3d.y;
	std::cout << "位姿测量值: x = " << pose_measurement[0] 
              << ", y = " << pose_measurement[1] << std::endl;
	kalman_filter_pose.update(pose_measurement);

	Eigen::VectorXd size_measurement(2);
	size_measurement << object3d.l, object3d.w;
	std::cout << "尺寸测量值: l = " << size_measurement[0] 
              << ", w = " << size_measurement[1] << std::endl;
	kalman_filter_size.update(size_measurement);

	float vel_yaw = norm_radian(atan2(object3d.vy, object3d.vx));
	Eigen::VectorXd yaw_measurement(2);
	yaw_measurement << object3d.yaw, vel_yaw;
	std::cout << "偏航测量值: yaw = " << yaw_measurement[0] 
              << ", vel_yaw = " << yaw_measurement[1] << std::endl;
	kalman_filter_yaw.update(yaw_measurement);

	state = TrackState::Tracked;
	lost_frame = 0;
	std::cout << "更新状态为 Tracked，丢失帧数重置为 0" << std::endl;
}

void STrack::predict()
{
	Eigen::VectorXd pose_pred = kalman_filter_pose.predict();
	object3d.x = pose_pred[0];
	object3d.y = pose_pred[1];
	object3d.vx = pose_pred[2];
	object3d.vy = pose_pred[3];
	Eigen::VectorXd size_pred = kalman_filter_size.predict();
	object3d.l = size_pred[0];
	object3d.w = size_pred[1];
	Eigen::VectorXd yaw_pred = kalman_filter_yaw.predict();
	object3d.yaw = yaw_pred[0];
	lost_frame++;
}

void STrack::mark_lost()
{
	state = TrackState::Lost;
}

void STrack::mark_removed()
{
	state = TrackState::Removed;
}

int STrack::next_id()
{
	static int _count = 0;
	_count++;
	return _count;
}