# jethexa_ros
This repository is part of my class project (Introduction to Autonomous Mobile Systems - EEL 5683) at the Intelligent Systems and Robotics Department at UWF.

## Jethexa Intro
<p align="center">
<img src="media/image.png" width="400">
<img src="media/Image2.png" width="400">
</p>

## Jethexa Communication
<p align="center">
<img src="media/jethexa_com.png" width="400">
</p>

## Sensor Only Published Topics
```
/jethexa/battery_low_alarm
/jethexa/body_link_to_laser_link
/jethexa/camera/camera_nodelet_manager
/jethexa/camera/depth_metric
/jethexa/camera/depth_metric_rect
/jethexa/camera/depth_points
/jethexa/camera/depth_rectify_depth
/jethexa/camera/depth_registered_sw_metric_rect
/jethexa/camera/driver
/jethexa/camera/points_xyzrgb_sw_registered
/jethexa/camera/register_depth_rgb
/jethexa/camera/rgb_rectify_color
/jethexa/camera/uvc_color
/jethexa/camera_base_link
/jethexa/camera_base_link1
/jethexa/camera_base_link2
/jethexa/camera_base_link3
/jethexa/camera_link_2_base_link_1
/jethexa/ekf_se
/jethexa/jethexa_controller
/jethexa/joystick_control
/jethexa/rf2o_laser_odometry
/jethexa/robot_state_publisher
/jethexa/ydlidar_lidar_g4_publisher
/mapping_rviz
/rosout
```

## Jethexa Autonomous Waypoint Navigation
### Transform Tree
<p align="center">
<img src="media/jethexa_tf.png" width="1000">
</p>

### Published Topics
```
/jethexa/cmd_vel
/jethexa/imu/filtered
/jethexa/jethexa_controller/cmd_vel
/jethexa/jethexa_controller/pose_transform_euler
/jethexa/jethexa_controller/run_actionset
/jethexa/jethexa_controller/set_head_absolute
/jethexa/jethexa_controller/set_head_relatively
/jethexa/jethexa_controller/set_leg_absolute
/jethexa/jethexa_controller/set_leg_relatively
/jethexa/jethexa_controller/set_pose
/jethexa/jethexa_controller/set_pose_euler
/jethexa/jethexa_controller/traveling
/jethexa/joint_states
/jethexa/odom/filtered
/jethexa/odom/laser
/jethexa/odom/raw
/jethexa/point_cloud
/jethexa/rtabmap/cloud_ground
/jethexa/rtabmap/cloud_map
/jethexa/rtabmap/cloud_obstacles
/jethexa/rtabmap/global_path
/jethexa/rtabmap/global_path_nodes
/jethexa/rtabmap/global_pose
/jethexa/rtabmap/goal
/jethexa/rtabmap/goal_node
/jethexa/rtabmap/goal_out
/jethexa/rtabmap/goal_reached
/jethexa/rtabmap/gps/fix
/jethexa/rtabmap/grid_prob_map
/jethexa/rtabmap/grid_prob_map_updates
/jethexa/rtabmap/imu
/jethexa/rtabmap/info
/jethexa/rtabmap/initialpose
/jethexa/rtabmap/labels
/jethexa/rtabmap/landmarks
/jethexa/rtabmap/local_grid_empty
/jethexa/rtabmap/local_grid_ground
/jethexa/rtabmap/local_grid_obstacle
/jethexa/rtabmap/local_path
/jethexa/rtabmap/local_path_nodes
/jethexa/rtabmap/localization_pose
/jethexa/rtabmap/mapData
/jethexa/rtabmap/mapGraph
/jethexa/rtabmap/mapOdomCache
/jethexa/rtabmap/mapPath
/jethexa/rtabmap/octomap_binary
/jethexa/rtabmap/octomap_empty_space
/jethexa/rtabmap/octomap_full
/jethexa/rtabmap/octomap_global_frontier_space
/jethexa/rtabmap/octomap_grid
/jethexa/rtabmap/octomap_ground
/jethexa/rtabmap/octomap_obstacles
/jethexa/rtabmap/octomap_occupied_space
/jethexa/rtabmap/proj_map
/jethexa/rtabmap/republish_node_data
/jethexa/rtabmap/scan_map
/jethexa/rtabmap/tag_detections
/jethexa/rtabmap/user_data_async
/jethexa/scan
/jethexa/voltage
/map
/rosout
/rosout_agg
/rtabmap/republish_node_data
/tf
/tf_static
```

## Node List
```
/jethexa/battery_low_alarm
/jethexa/body_link_to_laser_link
/jethexa/camera/camera_nodelet_manager
/jethexa/camera/depth_metric
/jethexa/camera/depth_metric_rect
/jethexa/camera/depth_points
/jethexa/camera/depth_rectify_depth
/jethexa/camera/depth_registered_sw_metric_rect
/jethexa/camera/driver
/jethexa/camera/points_xyzrgb_sw_registered
/jethexa/camera/register_depth_rgb
/jethexa/camera/rgb_rectify_color
/jethexa/camera/uvc_color
/jethexa/camera_base_link
/jethexa/camera_base_link1
/jethexa/camera_base_link2
/jethexa/camera_base_link3
/jethexa/camera_link_2_base_link_1
/jethexa/ekf_se
/jethexa/jethexa_controller
/jethexa/joystick_control
/jethexa/rf2o_laser_odometry
/jethexa/robot_state_publisher
/jethexa/rtabmap/rtabmap
/jethexa/ydlidar_lidar_g4_publisher
/mapping_rviz
/rosout
```

## Jethexa Autonomous Waypoint Navigation
<p align="center">
<img src="media/image3.png" width="600">
</p>
