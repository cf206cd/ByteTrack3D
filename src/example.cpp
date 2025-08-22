#include <iostream>
#include <vector>
#include <random>
#include "ByteTrack.h"
#include "object.h"

using namespace std;

// 生成随机对象
vector<Object3D> generate_random_objects(int count, int frame_id) {
    vector<Object3D> objects;
    
    // 随机数生成器
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
        obj.class_id = i % 3 + 1; // 3个类别
        obj.score = score_dist(gen);
        objects.push_back(obj);
    }
    
    return objects;
}

// 模拟移动对象
vector<Object3D> move_objects(const vector<Object3D>& prev_objects, float dt = 0.1f) {
    vector<Object3D> new_objects;
    
    random_device rd;
    mt19937 gen(rd());
    normal_distribution<float> noise(0.0f, 0.1f);
    
    for (const auto& prev : prev_objects) {
        Object3D obj = prev;
        // 根据速度更新位置
        obj.x += obj.vx * dt + noise(gen);
        obj.y += obj.vy * dt + noise(gen);
        // 添加小的随机变化
        obj.w += noise(gen) * 0.1f;
        obj.l += noise(gen) * 0.1f;
        obj.score = max(0.5f, min(1.0f, obj.score + noise(gen) * 0.05f));
        new_objects.push_back(obj);
    }
    
    return new_objects;
}

int main() {
    cout << "ByteTrack3D 示例程序" << endl;
    cout << "===================" << endl << endl;
    
    // 创建跟踪器
    ByteTrack tracker;
    
    // 模拟10帧数据
    const int num_frames = 10;
    vector<Object3D> current_objects = generate_random_objects(5, 0);
    
    for (int frame = 1; frame <= num_frames; frame++) {
        cout << "第 " << frame << " 帧:" << endl;
        cout << "输入对象: " << current_objects.size() << " 个" << endl;
        
        // 更新跟踪器
        auto tracks = tracker.update(current_objects);
        
        cout << "跟踪结果: " << tracks.size() << " 个" << endl;
        for (const auto& track : tracks) {
            cout << "  ID: " << track.track_id 
                 << " 位置: (" << setprecision(2) << track.object3d.x 
                 << ", " << track.object3d.y << ")"
                 << " 速度: (" << track.object3d.vx 
                 << ", " << track.object3d.vy << ")"
                 << " 分数: " << track.object3d.score << endl;
        }
        
        // 生成下一帧对象
        if (frame % 3 == 0) {
            // 每3帧随机添加/删除一些对象
            random_device rd;
            mt19937 gen(rd());
            uniform_int_distribution<int> count_dist(3, 6);
            current_objects = generate_random_objects(count_dist(gen), frame);
        } else {
            // 正常移动对象
            current_objects = move_objects(current_objects);
        }
        
        cout << endl;
    }
    
    cout << "示例完成！" << endl;
    cout << "使用说明:" << endl;
    cout << "1. 编译: cd ByteTrack3D && mkdir build && cd build && cmake .. && make" << endl;
    cout << "2. 运行: ./example" << endl;
    cout << "3. 运行测试: make test" << endl;
    
    return 0;
}