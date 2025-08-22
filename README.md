# ByteTrack3D

ByteTrack3D is a 3D object tracking library based on the ByteTrack algorithm, extended for 3D object tracking applications. It is designed for tracking 3D objects in autonomous driving and robotics scenarios.

## Features

- 3D object tracking with extended Kalman filters
- Support for 3D bounding boxes with position, size, velocity, and orientation
- Multi-object tracking using the ByteTrack association algorithm
- Modular design with separate components for tracking, filtering, and association

## Dependencies

- OpenCV
- Eigen3
- CMake (>= 3.10)

## Building

```bash
mkdir build
cd build
cmake ..
make
```

## Installation

You can install the library system-wide using the provided installation script:

```bash
./install.sh
```

This will compile and install the library to `/usr/local/lib` and headers to `/usr/local/include`.

Alternatively, you can manually install:

```bash
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make
sudo make install
```

## Usage

After building, you can run the example:

```bash
./example
```

### Integration

To use ByteTrack3D in your own project:

1. Include the header:
```cpp
#include "ByteTrack.h"
```

2. Create a tracker instance:
```cpp
ByteTrack tracker;
```

3. Initialize tracks with first frame detections:
```cpp
std::vector<Object3D> first_frame_detections = get_first_frame_detections();
std::vector<STrack> initial_tracks = tracker.init(first_frame_detections);
```

4. For each subsequent frame, update the tracker with new detections:
```cpp
std::vector<Object3D> detections = get_detections(); // Your detection function
std::vector<STrack> tracks = tracker.update(detections);
```

### CMake Integration

If you're using CMake for your project, you can integrate ByteTrack3D by adding these lines to your `CMakeLists.txt`:

```cmake
find_library(BYTETRACK3D_LIBRARY ByteTrack3D)
find_path(BYTETRACK3D_INCLUDE_DIR ByteTrack.h)

target_link_libraries(your_target ${BYTETRACK3D_LIBRARY})
target_include_directories(your_target PRIVATE ${BYTETRACK3D_INCLUDE_DIR})
```

### Object3D Structure

The `Object3D` structure contains the following fields:
- `x`, `y`: Position coordinates
- `w`, `l`: Width and length of the object
- `vx`, `vy`: Velocity components
- `yaw`: Orientation angle
- `class_id`: Object class identifier
- `score`: Detection confidence score

## API Reference

### ByteTrack Class

- `ByteTrack(int max_time_lost, float detect_thresh, float high_thresh, float match_thresh_1, float match_thresh_2)`: Constructor with configurable parameters
- `std::vector<STrack> update(std::vector<Object3D> objects)`: Update the tracker with new detections
- `std::vector<STrack> init(std::vector<Object3D> objects)`: Initialize tracks with detections

### STrack Class

Represents a single track with the following properties:
- `object3d`: The tracked 3D object
- `track_id`: Unique identifier for the track
- `state`: Current state (New, Tracked, Lost, Removed)
- `lost_frame`: Number of frames since last association

## Algorithm

ByteTrack3D extends the original ByteTrack algorithm for 3D object tracking:

1. Prediction step using extended Kalman filters for position, size, and orientation
2. Association using IoU-based matching with multiple thresholds
3. Track management with lost and removed states

The implementation uses separate Kalman filters for different aspects of the 3D object:
- `EKF_CTRA`: For position and velocity
- `KF_SIZE`: For width and length
- `KF_YAW`: For orientation angle

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.