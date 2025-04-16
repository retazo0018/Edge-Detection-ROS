# Edge Detection ROS
This ROS package detects edges in a checkerboard image, projects those edges into 3D space, and visualizes them as markers for each TF frame. 

## Description
- Implemented in ROS1 Noetic
- C++ and Python is supported
- Noise removal using Bilateral Smoothing
- Edge Detection using Canny Edge Detection
    - The gradient magnitude and orientation are calculated to identify areas of high intensity change, indicating potential edges.
- ROS service to detect edges for images located in a directory
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
- Visualization of 3D edges as RViz markers for each TF frame in RviZ

## Getting started

### Basic
- `src/EdgeDetector.cpp` and `include/EdgeDetector.hpp` contains the source code for C++ Implementation.
- `src/edge_detector.py` contains the source code for Python Implementation.
- Required packages include opencv and numpy.

### Vision_ROS
- Start `roscore` on a terminal.
- Intialise a ROS workspace, copy this repository contents inside `src` directory
- Workspace Setup Step
    - On another terminal, run `catkin_make` from the root of the catkin workspace.
    - Source the workspace `source <path to catkin workspace>/devel/setup.bash`.
- Install any missing ROS packages when prompted by `sudo apt update && sudo apt install ros-noetic-<package-name>`
- Start the server/node using `rosrun edge_detection edge_detection_bin` in C++ or `rosrun edge_detection edge_detector.py` in Python.

- ROS Service
    - On a new terminal, start the client using `rosrun edge_detection edge_detector_client.py "/<PATH_TO_IMAGES_DIRECTORY>/"`.
    - A superimposed image with the detected edges highlighted in green is saved for each input image, inside a `results` subdirectory located within the same input directory.

- Bagfile Input
    - On a different terminal, play the bagfile by `rosbag play --clock -l <path to bagfile>`.
    - The default values of the camera topics are listed below. Please modify otherwise.
        - `camera_depth_aligned_pc_topic`: `/camera/depth/points`
        - `camera_depth_image_topic`: `/camera/depth/image_rect_raw`
        - `camera_rgb_image_topic`: `/camera/color/image_raw`
    - The edge image and corresponding point cloud are published to the `/edge_image` and `/edge_points` topics, respectively.

### Robot_ROS
- Start `roscore` on a terminal.
- Workspace Setup Step
    - On another terminal, run `catkin_make` from the root of the catkin workspace.
    - Source the workspace `source <path to catkin workspace>/devel/setup.bash`.
- Start the node using `rosrun edge_detection edge_detection_bin` for C++ or `rosrun edge_detection edge_detector.py` for python.
- On a new terminal, repeat the workspace setup step and launch the robot using `rosparam set /use_sim_time true && roslaunch mira_picker display.launch gripper_name:=robotiq2f_140 publish_joint_state:=false publish_robot_state:=false`. A RViz window opens.
- On a different terminal, play the bagfile by `rosbag play --clock -l <path to bagfile>`.
- Edges are visualized as markers in the `/edge_points_marker` topic. 
- Visualize `/edge_points_marker` topic in Rviz to see edges as markers along with the robot.

## Notes
- The result videos are found in `results` directory.
    - The video `results/result_video_c++.mp4` shows the result video using C++ node. 
    - The video `results/result_video_py.mp4` shows the result video using python node. 
- The results of sample images are found inside `data/results` directory.
