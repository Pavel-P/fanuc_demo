#Setup

Required packages:
- fanuc
- fanuc_experimental (forked)
- gazebo_ros_pkgs
- descartes (forked)
- gazebo, version 5
- libgazebo5-dev
- moveit

#Use

To run:

 1. ```source scripts/update_model_path.bash```
 2. ```rosrun fanuc_demo update_model_sdf.bash```
 3. ```roslaunch fanuc_demo fanuc.launch```

Check fanuc.launch and demo.launch for additional options.
