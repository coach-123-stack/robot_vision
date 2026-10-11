# robot_vision

ROS1 package for ArUco marker detection and pose estimation.

Features:
- subscribes to camera image and camera_info
- detects ArUco markers
- estimates each marker pose relative to the camera
- publishes `geometry_msgs/PoseStamped` and `geometry_msgs/PoseArray`
- broadcasts TF transforms for each marker
- publishes visualization markers for RViz

## Package layout

- `src/aruco_pose_node.cpp` — main ArUco detection node
- `launch/aruco_pose.launch` — launch example
- `README.md` — usage guide

## Quick start

1. Create a catkin workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

2. Clone this package

```bash
git clone https://github.com/coach-123-stack/robot_vision.git
```

3. Build

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

4. Run the node

```bash
roslaunch robot_vision aruco_pose.launch
```

5. Check topics

```bash
rostopic list | grep aruco
rostopic echo /aruco_pose
```

## Parameters

Available launch arguments:
- `image_topic` — default `/camera/image_raw`
- `camera_info_topic` — default `/camera/camera_info`
- `camera_frame` — default `camera`
- `marker_size` — default `0.08` (meters)
- `dictionary` — default `DICT_6X6_250`

## Marker generation

Use an ArUco marker generator such as:
- https://chev.me/arucogen/

## Typical output

The node publishes:
- `/aruco_pose` — `geometry_msgs/PoseStamped` for the newest detection
- `/aruco_pose_array` — `geometry_msgs/PoseArray`
- `/aruco_markers` — `visualization_msgs/MarkerArray`
- TF: `camera -> aruco_marker_<id>`

## Notes

- A valid camera calibration (`/camera/camera_info`) is required.
- The marker size should match the physical size of the printed marker.
