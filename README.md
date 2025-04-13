# Edge Detection ROS
This ROS package detects edges in a checkerboard image, projects those edges into 3D space, and visualizes them as markers for each TF frame.

## Features
- Implemented in ROS1 Noetic
- Artifacts removal using Bilateral Smoothing
- Edge Detection using Canny
- Outlier Removal using Largest Contour 
    - Largest contour by area is identified and any detected edge outside the largest contour is removed to retain only edges of the checkerboard.
- Projection of detected edges into 3D space using camera intrinsics
- Visualization of 3D edges as RViz markers for each TF frame
    - A timer callback is triggered every 0.1 seconds to publish the markers for each frame. For more real-time updates, change this value to 0.05 or lower.

## Getting started

### Basic
- `src/edge_detection.py` contains the source code.
- Required packages include opencv and numpy.

### Vision_ROS
- Start `roscore` on a terminal.
- Intialise a ROS workspace and run `catkin_make` from root.
- Install any missing ROS packages when prompted by `sudo apt update && sudo apt install ros-noetic-<package-name>`
- Source the workspace `source <path to catkin workspace>/devel/setup.bash`.
- Start the server/node using `rosrun edge_detection edge_detector.py`.

- ROS Service
    - Start the client using `rosrun edge_detection edge_detector_client.py "/<PATH_TO_IMAGES_DIRECTORY>/"`.
    - A superimposed image with the detected edges highlighted in green is saved for each input image, inside a `results` subdirectory located within the same input directory.

- Bagfile Input
    - Play the bagfile by `rosbag play --clock -l <path to bagfile>`.
    - The default values of the camera topics are listed below. Please modify otherwise.
        - `camera_depth_aligned_pc_topic`: `/camera/depth/points`
        - `camera_depth_image_topic`: `/camera/depth/image_rect_raw`
        - `camera_rgb_image_topic`: `/camera/color/image_raw`
    - Open RViz and change the fixed frame to `camera_color_optical_frame`.
    - The edge image and corresponding point cloud are published to the `/edge_image` and `/edge_points` topics, respectively.

### Robot_ROS
- Start `roscore` on a terminal.
- Source the workspace `source <path to catkin workspace>/devel/setup.bash`.
- Start the node using `rosrun edge_detection edge_detector.py`.
- Launch the robot using `rosparam set /use_sim_time true && roslaunch mira_picker display.launch gripper_name:=robotiq2f_140 publish_joint_state:=false publish_robot_state:=false`
- Play the bagfile by `rosbag play --clock -l <path to bagfile>`.
- Edges are visualized as markers in the `/edge_points_marker` topic. In RViz, set the fixed frame to any TF frame of your choice to view the markers in the desired frame.

## Notes
- The result videos are found in `results` directory.
    - The video `results/result_camera_frame.mp4` visualizes the output of ROBOT_ROS package in `camera_color_optical_frame` only.
    - The video `results/result_all_frames_fast_bag_playback.mp4` visualizes the output of ROBOT_ROS package in a few arbitrary frames. This video was recorded while setting the `marker_timer_callback_freq` parameter to `0.05`.
    - The video `results/result_all_frames_slow_bag_playback.mp4` visualizes the output of ROBOT_ROS package in a few arbitrary frames. This video was recorded while setting the `marker_timer_callback_freq` parameter to `0.1` and playing the bag file at a slower rate of `0.5`.
- The results of sample images are found inside `data/results` directory.

## Areas of Improvement
- A slight lag is observed when publishing marker points in each frame, indicating a need for further optimization.
- Reimplement the ROS node in C++ to evaluate whether the lag persists and to potentially improve performance.
