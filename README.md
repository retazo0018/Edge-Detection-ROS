# Edge Detection ROS
This ROS package detects edges in a checkerboard image, projects those edges into 3D space, and visualizes them as markers for each TF frame.
⚠️ **Note:** The branch `feat/cpp` contains the C++ Implementation and is currently a WIP. 

## Description
- Implemented in ROS1 Noetic
- Noise removal using Bilateral Smoothing
- Edge Detection using Canny Edge Detection
    - The gradient magnitude and orientation are calculated to identify areas of high intensity change, indicating potential edges.
- Outlier Removal using Largest Contour 
    - Largest contour by area is identified and any detected edge outside the largest contour is removed to retain only edges of the checkerboard.
- Projection of detected edges into 3D space using formula below,
    - X = ((u - cx) * Z) / fx
    - Y = ((v - cy) * Z) / fy
    - Z = d
    - where,
        - u, v are the pixel coordinates in the image
        - d is the depth value at that pixel
        - cx, cy is the principal point of the camera (intrinsic parameters)
        - fx, fy are the focal lengths in x and y directions
        - (X, Y, Z) is the 3D point in the camera coordinate system
- Visualization of 3D edges as RViz markers for each TF frame
    - A timer callback is triggered every 0.05 seconds to publish the markers for each frame. For more real-time updates, change the value of `marker_timer_callback_freq` parameter in `src/edge_detector.py` to 0.01 or lower. Increase this value to 0.1 for slightly more stable marker updates at the cost of a slight lag.
    - The 3D edge points are downsampled by the value of `edge_downsampling_rate` parameter in `src/edge_detector.py` to achieve real-time marker updates. The value is by default set to 5. A higher value will lead to limited 3D edge markers.

## Getting started

### Basic
- `src/edge_detection.py` contains the source code.
- Required packages include opencv and numpy.

### Vision_ROS
- Start `roscore` on a terminal.
- Intialise a ROS workspace, copy this repository contents inside `src` directory and run `catkin_make` from root of the workspace.
- Install any missing ROS packages when prompted by `sudo apt update && sudo apt install ros-noetic-<package-name>`
- On another terminal, source the workspace `source <path to catkin workspace>/devel/setup.bash`.
- Start the server/node using `rosrun edge_detection edge_detector.py`.

- ROS Service
    - On a new terminal, start the client using `rosrun edge_detection edge_detector_client.py "/<PATH_TO_IMAGES_DIRECTORY>/"`.
    - A superimposed image with the detected edges highlighted in green is saved for each input image, inside a `results` subdirectory located within the same input directory.

- Bagfile Input
    - On a different terminal, play the bagfile by `rosbag play --clock -l <path to bagfile>`.
    - The default values of the camera topics are listed below. Please modify otherwise.
        - `camera_depth_aligned_pc_topic`: `/camera/depth/points`
        - `camera_depth_image_topic`: `/camera/depth/image_rect_raw`
        - `camera_rgb_image_topic`: `/camera/color/image_raw`
    - Open RViz and change the fixed frame to `camera_color_optical_frame`.
    - The edge image and corresponding point cloud are published to the `/edge_image` and `/edge_points` topics, respectively.

### Robot_ROS
- Start `roscore` on a terminal.
- On another terminal, source the workspace `source <path to catkin workspace>/devel/setup.bash`.
- Start the node using `rosrun edge_detection edge_detector.py`.
- On a new terminal, launch the robot using `rosparam set /use_sim_time true && roslaunch mira_picker display.launch gripper_name:=robotiq2f_140 publish_joint_state:=false publish_robot_state:=false`
- On a different terminal, play the bagfile by `rosbag play --clock -l <path to bagfile>`.
- Edges are visualized as markers in the `/edge_points_marker` topic. In RViz, set the fixed frame to any TF frame of your choice to view the markers in the desired frame.

## Notes
- The result videos are found in `results` directory.
    - The video `results/result_camera_frame.mp4` visualizes the output of ROBOT_ROS package in `camera_color_optical_frame` only.
    - The video `results/result_all_frames_fast_bag_playback.mp4` visualizes the output of ROBOT_ROS package in a few arbitrary frames. This video was recorded while setting the `marker_timer_callback_freq` parameter to `0.05`.
    - The video `results/result_all_frames_slow_bag_playback.mp4` visualizes the output of ROBOT_ROS package in a few arbitrary frames. This video was recorded while setting the `marker_timer_callback_freq` parameter to `0.1` and playing the bag file at a slower rate of `0.5`.
- The results of sample images are found inside `data/results` directory.

## Areas of Improvement
- A slight lag is observed when publishing marker points in each frame, indicating a need for small further optimization.
- Reimplement the ROS node in C++ to evaluate whether the lag persists and to potentially improve performance.
