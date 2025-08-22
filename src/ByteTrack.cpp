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
    cout << "ByteTrack constructor called, max_time_lost: " << max_time_lost 
         << ", detect_thresh: " << detect_thresh 
         << ", high_thresh: " << high_thresh 
         << ", match_thresh_1: " << match_thresh_1 
         << ", match_thresh_2: " << match_thresh_2 << endl;
}

ByteTrack::~ByteTrack()
{
    cout << "ByteTrack destructor called" << endl;
}

std::vector<STrack> ByteTrack::init(std::vector<Object3D> objects)
{
    cout << "Starting initialization, detected " << objects.size() << " objects" << endl;
    for(int i = 0; i < objects.size(); i++)
    {
        stracks.push_back(STrack(objects[i]));
        cout << "Initializing STrack " << i << endl;
    }
    cout << "Initialization completed, current STrack count: " << stracks.size() << endl;
    return stracks;
}

std::vector<STrack> ByteTrack::update(std::vector<Object3D> objects)
{
    cout << "Starting update for frame " << frame_id + 1 << ", detected " << objects.size() << " objects" << endl;
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
    cout << "High score objects: " << high_score_objects.size() << ", Low score objects: " << low_score_objects.size() << endl;

    cout << "Starting prediction for all STrack states" << endl;
    for(int i=0;i<stracks.size();i++)
    {
       stracks[i].predict();
    }
    cout << "Prediction completed, total STrack count: " << stracks.size() << endl;

    vector<Object3D> tracks;
    for(int i=0;i<stracks.size();i++)
    {
        if(stracks[i].state != TrackState::Removed)
        {
            tracks.push_back(stracks[i].object3d);
        }
    }
    cout << "Filtered out unremoved STrack objects, count: " << tracks.size() << endl;

    vector<vector<int>> matches;
    vector<int> remain_tracks_idx;
    vector<int> remain_detections_idx;
    cout << "Starting first matching, matching threshold: " << match_thresh_1 << endl;
    match(tracks, high_score_objects, matches, remain_tracks_idx, remain_detections_idx, match_thresh_1);
    cout << "First matching completed, match pairs: " << matches.size() 
         << ", remaining track indices: " << remain_tracks_idx.size() 
         << ", remaining detection indices: " << remain_detections_idx.size() << endl;

    for(int i=0;i<matches.size();i++)
    {
        cout << "Updating STrack " << matches[i][0] << " with high score object " << matches[i][1] << endl;
        stracks[matches[i][0]].update(high_score_objects[matches[i][1]]);
    }

    vector<Object3D> remain_tracks;
    for(int i=0;i<remain_tracks_idx.size();i++)
    {
        remain_tracks.push_back(stracks[remain_tracks_idx[i]].object3d);
    }
    cout << "Getting objects for remaining tracks, count: " << remain_tracks.size() << endl;

    vector<vector<int>> re_matches;
    vector<int> re_remain_tracks_idx;
    vector<int> re_remain_detections_idx;
    cout << "Starting second matching, matching threshold: " << match_thresh_2 << endl;
    match(remain_tracks, low_score_objects, re_matches, re_remain_tracks_idx, re_remain_detections_idx, match_thresh_2);
    cout << "Second matching completed, match pairs: " << re_matches.size() 
         << ", remaining track indices: " << re_remain_tracks_idx.size() 
         << ", remaining detection indices: " << re_remain_detections_idx.size() << endl;

    for(int i=0;i<re_matches.size();i++)
    {
        stracks[remain_tracks_idx[re_matches[i][0]]].update(low_score_objects[re_matches[i][1]]);
        cout << "Updating STrack " << remain_tracks_idx[re_matches[i][0]] << " with low score object " << re_matches[i][1] << endl;
    }

    for(int i=0;i<re_remain_tracks_idx.size();i++)
    {
        stracks[re_remain_tracks_idx[i]].mark_lost();
        cout << "Marking STrack " << re_remain_tracks_idx[i] << " as lost" << endl;
    }

    for(int i=0;i<remain_detections_idx.size();i++)
    {
        stracks.push_back(STrack(high_score_objects[remain_detections_idx[i]]));
        cout << "Adding new STrack " << stracks.size() - 1 << " with high score object " << remain_detections_idx[i] << endl;
    }

    int removed_count = 0;
    for(int i=0;i<stracks.size();i++)
    {
        if(stracks[i].lost_frame > max_time_lost)
        {
            stracks[i].mark_removed();
            removed_count++;
            cout << "Marking STrack " << i << " as removed, lost frames: " << stracks[i].lost_frame << endl;
        }
    }
    cout << "Marked " << removed_count << " STracks as removed" << endl;

    // Use std::remove_if and erase combination to remove elements with Removed state for better code efficiency
    int before_size = stracks.size();
    stracks.erase(std::remove_if(stracks.begin(), stracks.end(), [](const STrack& track) {
        return track.state == TrackState::Removed;
    }), stracks.end());
    cout << "Removed STracks with removed state, count before: " << before_size << ", count after: " << stracks.size() << endl;

    cout << "Frame " << frame_id << " update completed, current STrack count: " << stracks.size() << endl;
    return stracks;
}
