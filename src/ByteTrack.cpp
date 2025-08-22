#include "ByteTrack.h"
#include <iostream>
using namespace std;


ByteTrack::ByteTrack(int max_time_lost, float detect_thresh, float high_thresh, float match_thresh_1, float match_thresh_2)
    : frame_id(0),
      max_time_lost(max_time_lost),
      detect_thresh(detect_thresh),
      high_thresh(high_thresh),
      match_thresh_1(match_thresh_1),
      match_thresh_2(match_thresh_2)
{
    cout << "ByteTrack 构造函数被调用，max_time_lost: " << max_time_lost 
         << ", detect_thresh: " << detect_thresh 
         << ", high_thresh: " << high_thresh 
         << ", match_thresh_1: " << match_thresh_1 
         << ", match_thresh_2: " << match_thresh_2 << endl;
}

ByteTrack::~ByteTrack()
{
    cout << "ByteTrack 析构函数被调用" << endl;
}

std::vector<STrack> ByteTrack::init(std::vector<Object3D> objects)
{
    cout << "开始初始化，检测到 " << objects.size() << " 个目标" << endl;
    for(int i = 0; i < objects.size(); i++)
    {
        stracks.push_back(STrack(objects[i]));
        cout << "初始化第 " << i << " 个 STrack" << endl;
    }
    cout << "初始化完成，当前共有 " << stracks.size() << " 个 STrack" << endl;
    return stracks;
}

std::vector<STrack> ByteTrack::update(std::vector<Object3D> objects)
{
    cout << "开始第 " << frame_id + 1 << " 帧的更新，检测到 " << objects.size() << " 个目标" << endl;
    frame_id++;
    vector<Object3D> high_score_objects;
    vector<Object3D> low_score_objects;
    for(int i=0;i<objects.size();i++)
    {
        if(objects[i].score > detect_thresh)
        {
            high_score_objects.push_back(objects[i]);
        }
        else
        {
            low_score_objects.push_back(objects[i]);
        }
    }
    cout << "高分目标数量: " << high_score_objects.size() << "，低分目标数量: " << low_score_objects.size() << endl;

    cout << "开始预测所有 STrack 的状态" << endl;
    for(int i=0;i<stracks.size();i++)
    {
       stracks[i].predict();
    }
    cout << "预测完成，共 " << stracks.size() << " 个 STrack" << endl;

    vector<Object3D> tracks;
    for(int i=0;i<stracks.size();i++)
    {
        if(stracks[i].state != TrackState::Removed)
        {
            tracks.push_back(stracks[i].object3d);
        }
    }
    cout << "筛选出未移除的 STrack 对应的目标，数量: " << tracks.size() << endl;

    vector<vector<int>> matches;
    vector<int> remain_tracks_idx;
    vector<int> remain_detections_idx;
    cout << "开始第一次匹配，匹配阈值: " << match_thresh_1 << endl;
    match(tracks, high_score_objects, matches, remain_tracks_idx, remain_detections_idx, match_thresh_1);
    cout << "第一次匹配完成，匹配对数: " << matches.size() 
         << "，剩余轨迹索引数量: " << remain_tracks_idx.size() 
         << "，剩余检测索引数量: " << remain_detections_idx.size() << endl;

    for(int i=0;i<matches.size();i++)
    {
        cout << "更新第 " << matches[i][0] << " 个 STrack，使用第 " << matches[i][1] << " 个高分目标" << endl;
        stracks[matches[i][0]].update(high_score_objects[matches[i][1]]);
    }

    vector<Object3D> remain_tracks;
    for(int i=0;i<remain_tracks_idx.size();i++)
    {
        remain_tracks.push_back(stracks[remain_tracks_idx[i]].object3d);
    }
    cout << "获取剩余轨迹对应的目标，数量: " << remain_tracks.size() << endl;

    vector<vector<int>> re_matches;
    vector<int> re_remain_tracks_idx;
    vector<int> re_remain_detections_idx;
    cout << "开始第二次匹配，匹配阈值: " << match_thresh_2 << endl;
    match(remain_tracks, low_score_objects, re_matches, re_remain_tracks_idx, re_remain_detections_idx, match_thresh_2);
    cout << "第二次匹配完成，匹配对数: " << re_matches.size() 
         << "，剩余轨迹索引数量: " << re_remain_tracks_idx.size() 
         << "，剩余检测索引数量: " << re_remain_detections_idx.size() << endl;

    for(int i=0;i<re_matches.size();i++)
    {
        stracks[remain_tracks_idx[re_matches[i][0]]].update(low_score_objects[re_matches[i][1]]);
        cout << "更新第 " << remain_tracks_idx[re_matches[i][0]] << " 个 STrack，使用第 " << re_matches[i][1] << " 个低分目标" << endl;
    }

    for(int i=0;i<re_remain_tracks_idx.size();i++)
    {
        stracks[re_remain_tracks_idx[i]].mark_lost();
        cout << "标记第 " << re_remain_tracks_idx[i] << " 个 STrack 为丢失状态" << endl;
    }

    for(int i=0;i<remain_detections_idx.size();i++)
    {
        stracks.push_back(STrack(high_score_objects[remain_detections_idx[i]]));
        cout << "新增第 " << stracks.size() - 1 << " 个 STrack，使用第 " << remain_detections_idx[i] << " 个高分目标" << endl;
    }

    int removed_count = 0;
    for(int i=0;i<stracks.size();i++)
    {
        if(stracks[i].lost_frame > max_time_lost)
        {
            stracks[i].mark_removed();
            removed_count++;
            cout << "标记第 " << i << " 个 STrack 为移除状态，丢失帧数: " << stracks[i].lost_frame << endl;
        }
    }
    cout << "共标记 " << removed_count << " 个 STrack 为移除状态" << endl;

    // 使用标准库的 remove_if 和 erase 组合来移除状态为 Removed 的元素，提高代码效率
    int before_size = stracks.size();
    stracks.erase(std::remove_if(stracks.begin(), stracks.end(), [](const STrack& track) {
        return track.state == TrackState::Removed;
    }), stracks.end());
    cout << "移除移除状态的 STrack，移除前数量: " << before_size << "，移除后数量: " << stracks.size() << endl;

    cout << "第 " << frame_id << " 帧更新完成，当前共有 " << stracks.size() << " 个 STrack" << endl;
    return stracks;
}
