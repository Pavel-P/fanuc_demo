#Setup

Required packages:
- fanuc
- fanuc_experimental (forked)
- descartes (forked)
- ROS Industrial packages
- moveit_full
- ros_control
- ros_controllers
- gazebo
- gazebo_ros_pkgs (source install)
- libgazebo5-dev

Optional:
- industrial_trajectory_filters
- fermi moveit_cartesian_motion_plugin (rkojcev fork)

#Use

To run:

 1. ```source scripts/update_model_path.bash```
 2. ```rosrun fanuc_demo update_model_sdf.bash```
 3. ```roslaunch fanuc_demo fanuc.launch```

Check fanuc.launch and demo.launch for additional options.
