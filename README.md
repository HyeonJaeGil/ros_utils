# ROS utils (ROS1 support only)

## rosbag and rostopic utils for reading and writing a rosbag file

Currently, this repository only includes an example to save timestamp.csv from rosbag.
Please refer to `./examples/rosbag2files.cpp`
```
rosrun ros_utils rosbag2files <path_to_rosbag> <path_to_save_timestamps>
```