#include <iostream>
#include <vector>
#include <random>
#include "ByteTrack.h"
#include "object.h"

using namespace std;

// Generate random objects
vector<Object3D> generate_random_objects(int count, int frame_id) {
    vector<Object3D> objects;
    
    // Random number generator
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<float> pos_dist(-10.0f, 10.0f);
    uniform_real_distribution<float> size_dist(1.0f, 5.0f);
    uniform_real_distribution<float> vel_dist(-2.0f, 2.0f);
    uniform_real_distribution<float> angle_dist(-M_PI, M_PI);
    uniform_real_distribution<float> score_dist(0.5f, 1.0f);
    
    for (int i = 0; i < count; i++) {
        Object3D obj;
        obj.x = pos_dist(gen);
        obj.y = pos_dist(gen);
        obj.w = size_dist(gen);
        obj.l = size_dist(gen);
        obj.vx = vel_dist(gen);
        obj.vy = vel_dist(gen);
        obj.yaw = angle_dist(gen);
        obj.class_id = i % 3 + 1; // 3 classes
        obj.score = score_dist(gen);
        objects.push_back(obj);
    }
    
    return objects;
}

// Simulate moving objects
vector<Object3D> move_objects(const vector<Object3D>& prev_objects, float dt = 0.1f) {
    vector<Object3D> new_objects;
    
    random_device rd;
    mt19937 gen(rd());
    normal_distribution<float> noise(0.0f, 0.1f);
    
    for (const auto& prev : prev_objects) {
        Object3D obj = prev;
        // Update position based on velocity
        obj.x += obj.vx * dt + noise(gen);
        obj.y += obj.vy * dt + noise(gen);
        // Add small random changes
        obj.w += noise(gen) * 0.1f;
        obj.l += noise(gen) * 0.1f;
        obj.score = max(0.5f, min(1.0f, obj.score + noise(gen) * 0.05f));
        new_objects.push_back(obj);
    }
    
    return new_objects;
}

int main() {
    cout << "ByteTrack3D Example Program" << endl;
    cout << "===========================" << endl << endl;
    
    // Create tracker
    ByteTrack tracker;
    
    // Initialize with first frame using init
    vector<Object3D> current_objects = generate_random_objects(5, 0);
    cout << "Frame 1 (Initialization):" << endl;
    cout << "Input objects: " << current_objects.size() << endl;
    
    auto tracks = tracker.init(current_objects);
    cout << "Initialization tracking result: " << tracks.size() << endl;
    for (const auto& track : tracks) {
        cout << "  ID: " << track.track_id 
             << " Position: (" << setprecision(2) << track.object3d.x 
             << ", " << track.object3d.y << ")"
             << " Velocity: (" << track.object3d.vx 
             << ", " << track.object3d.vy << ")"
             << " Score: " << track.object3d.score << endl;
    }
    cout << endl;
    
    // Simulate next 9 frames of data
    const int num_frames = 10;
    
    for (int frame = 2; frame <= num_frames; frame++) {
        cout << "Frame " << frame << ":" << endl;
        
        // Generate next frame objects
        if (frame % 4 == 0) {
            // Randomly add/remove some objects every 4 frames
            random_device rd;
            mt19937 gen(rd());
            uniform_int_distribution<int> count_dist(3, 6);
            current_objects = generate_random_objects(count_dist(gen), frame);
        } else {
            // Move objects normally
            current_objects = move_objects(current_objects);
        }
        
        cout << "Input objects: " << current_objects.size() << endl;
        
        // Update tracker
        tracks = tracker.update(current_objects);
        
        cout << "Tracking result: " << tracks.size() << endl;
        for (const auto& track : tracks) {
            cout << "  ID: " << track.track_id 
                 << " Position: (" << setprecision(2) << track.object3d.x 
                 << ", " << track.object3d.y << ")"
                 << " Velocity: (" << track.object3d.vx 
                 << ", " << track.object3d.vy << ")"
                 << " Score: " << track.object3d.score << endl;
        }
        
        cout << endl;
    }
    
    cout << "Example completed!" << endl;
    cout << "Usage instructions:" << endl;
    cout << "1. Compile: cd ByteTrack3D && mkdir build && cd build && cmake .. && make" << endl;
    cout << "2. Run: ./example" << endl;
    cout << "3. Run tests: make test" << endl;
    
    return 0;
}