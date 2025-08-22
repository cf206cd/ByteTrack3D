#pragma once
#include <vector>
using namespace std;
struct Object3D{
    float x;
    float y;
    float w;
    float l;
    float vx;
    float vy;
    float yaw;
    int class_id;
    float score;
};
float RoGDIoU(const Object3D& a, const Object3D& b, float w1=1.0, float w2=1.0);

void iou_distance(vector<Object3D> tracks, vector<Object3D> detections, vector<vector<float>>& dists);

double lapjv(const vector<vector<float> > &cost, vector<int> &rowsol, vector<int> &colsol,bool extend_cost, float cost_limit, bool return_cost);

void linear_assignment(vector<vector<float>> cost_matrix, float thresh,vector<vector<int> > &matches, vector<int> &unmatched_a, vector<int> &unmatched_b);

void match(vector<Object3D>& tracks, vector<Object3D>& detections, vector<vector<int>>& matches, vector<int>& unmatched_tracks, vector<int>& unmatched_detections, float match_thresh);
