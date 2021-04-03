# 3DoF tracking

```
roslaunch dynamic_robot_localization_tests test_bag_guardian_1.launch
roslaunch dynamic_robot_localization_tests test_bag_guardian_controller_3.launch
```

```
roslaunch dynamic_robot_localization_tests freiburg2_pioneer_360.launch
roslaunch dynamic_robot_localization_tests freiburg2_pioneer_slam1.launch
```



# 6 DoF tracking

```
roslaunch dynamic_robot_localization_tests ethzasl_kinect_dataset_high_complexity_slow_fly_movement.launch
```

or, manually each node

```
rosparam set /use_sim_time true
rosbag play /home/carloscosta/catkin_ws_3_perception/src/dynamic_robot_localization_tests/datasets/asl/ethzasl_kinect_dataset/0high-0slow-0fly-0_2011-02-19-11-44-41.bag --clock --pause --rate 1.0
```

```
rosrun rviz rviz -d /home/carloscosta/catkin_ws_3_perception/src/dynamic_robot_localization_tests/rviz/asl/dynamic_robot_localization_tridimensional.rviz
```

```
rosrun tf static_transform_publisher 0.0182521 -0.00901447 -0.0427566 0.508233 0.500217 0.510305 0.480699 vicon_vehicle_20 vicon_vehicle_20_corrected 10
rosrun tf static_transform_publisher 0 0 0 -0.5 0.5 -0.5 -0.5 vicon_vehicle_20_corrected base_link 10
rosrun tf static_transform_publisher 0 0 0 -0.5 0.5 -0.5 0.5 base_link openni_rgb_optical_frame 10
rosrun tf static_transform_publisher 0 0 0 0 0 0 1 base_link odom 10
```

```
roslaunch dynamic_robot_localization dynamic_robot_localization_system_6dof.launch sensor_frame_id:="openni_rgb_optical_frame" ambient_pointcloud_topic:="/camera/depth/points2" robot_initial_pose_in_base_to_map:=false robot_initial_x:=1.325 robot_initial_y:=-0.728 robot_initial_z:=1.633 robot_initial_roll:=0.035 robot_initial_pitch:=0.591 robot_initial_yaw:=-0.151 reference_pointcloud_filename:="/home/carloscosta/catkin_ws_3_perception/src/dynamic_robot_localization_tests/maps/asl/ethzasl_kinect_dataset/high-complexity-environment_icp-point-point_0.0025-voxel-grid_mls_0.005-rosbag-speed_preprocessed.ply" reference_pointcloud_available:=true
```



# 6 DoF feature pose estimation

```
roslaunch object_recognition_skill_server rviz.launch
roslaunch object_recognition_skill_server object_recognition.launch
```

```
rostopic pub --once /object_recognition/ObjectRecognitionSkill/goal
```

```
roslaunch object_recognition_dataset test_photoneo_s_0_pointcloud.launch
0 /home/carloscosta/catkin_ws_3_perception/src/object_recognition_dataset/models/psa/windage_tray__0.5mm_sag_2mm_step.ply
1 /home/carloscosta/catkin_ws_3_perception/src/object_recognition_dataset/models/simoldes/8080__0.5mm_sag_2mm_step.ply
2 /home/carloscosta/catkin_ws_3_perception/src/object_recognition_dataset/models/simoldes/8537__0.5mm_sag_2mm_step.ply
```

```
roslaunch object_recognition_fasten_dataset test_photoneo_s_45_pointcloud.launch
0 /home/carloscosta/catkin_ws_3_perception/src/object_recognition_fasten_dataset/models/thyssenkrupp/air_conditioning_grill__0.1mm_sag_0.5mm_step_meters_centered.ply
1 /home/carloscosta/catkin_ws_3_perception/src/object_recognition_fasten_dataset/models/embraer/reinforced_bracket__0.1mm_sag_0.5mm_step_meters.ply
2 /home/carloscosta/catkin_ws_3_perception/src/object_recognition_fasten_dataset/models/embraer/single_side_bracket__0.1mm_sag_0.5mm_step_meters.ply
3 /home/carloscosta/catkin_ws_3_perception/src/object_recognition_fasten_dataset/models/embraer/double_side_bracket__0.1mm_sag_0.5mm_step_meters.ply
```



# 6 DoF pca pose estimation

```
roslaunch pick_and_pack rviz.launch
roslaunch pick_and_pack object_recognition.launch __ns:="yaskawa"
roslaunch pick_and_pack tfs_photoneo.launch
roslaunch pick_and_pack tfs_photoneo_pneumatic_grippers_platform.launch
rosrun tf static_transform_publisher -0.380200 0.487150 0.573600 0.024355 0.976644 0.002237 -0.213468 base_link tool0 100
```

```
rostopic pub --once /yaskawa/object_recognition/ObjectRecognitionSkill/goal
```

```
rosrun point_cloud_io read _file_path:="/media/carloscosta/Docs/Dropbox/INESC/Projects/Scalable/ObjectRecognition/Tests/PackingSimoldesV2/PackingPlatform_Pneumatics/2020-12-17/Yaskawa HC10 in non collaborative mode/scans/point_cloud_0_camera_optical_frame.ply" _topic:=/camera/depth_registered/points _frame:=camera_optical_frame
0 /home/carloscosta/catkin_ws_7_demonstrations/src/pick_and_pack/models/scalable/8477_left_scan.ply
1 /home/carloscosta/catkin_ws_7_demonstrations/src/pick_and_pack/models/scalable/8477_right_scan.ply
```
