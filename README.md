#Setup

Required packages:
- fanuc
- fanuc_experimental (forked)
- gazebo_ros_pkgs
- descartes
- gazebo
- libgazebo5-dev
- moveit

#Use

Runs similar to lwr_test:
 1. ```source update_model_path.bash```
 2. ```rosrun fanuc_demo update_model_sdf.bash```
 3. ```roslaunch fanuc_demo fanuc.launch load_moveit:=true```
 4. ```roslaunch fanuc_demo demo.launch``` (in seperate window)
