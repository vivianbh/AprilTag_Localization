robot_localization
==================

`robot_localization` is a package of nonlinear state estimation nodes. 
The package was developed by Charles River Analytics, Inc. Please see documentation here: http://docs.ros.org/melodic/api/robot_localization/html/index.html


How to run

``` bash=
// environment
$ roslaunch rotors_gazebo iris_one_exp.launch

// apriltag detection
$ roslaunch apriltag_ros continuous_detection.launch

$ rosrun robot_localization apriltag_coordinate_transformation
$ rosrun robot_localization imu_coord_enu

// fusion one imu and tags' position
$ roslaunch robot_localization ekf_localization.launch

$ roslaunch rotors_gazebo controller_geometry_iris_exp.launch
```

Fusion data is on topic `/odometry/filtered`
